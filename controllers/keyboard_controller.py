"""Keyboard -> bounded velocity commands only. No robot pose/network calls."""
from pathlib import Path
import json,math

PROJECT_ROOT=Path(r'D:\unitree_g1')

class KeyboardController:
    def __init__(self,config,deploy):
        self.config=config
        self.mapping=dict(config['keyboard_mapping'])
        ranges=deploy['commands']['base_velocity']['ranges']
        self.low=[ranges[k][0] for k in ['lin_vel_x','lin_vel_y','ang_vel_z']]
        self.high=[ranges[k][1] for k in ['lin_vel_x','lin_vel_y','ang_vel_z']]
        self.speeds=[config['max_forward_velocity'],config['max_backward_velocity'],config['max_lateral_velocity'],config['max_yaw_rate']]
        if not all(math.isfinite(v) and v>0 for v in self.speeds): raise ValueError('Invalid keyboard speed')
        forward,backward,lateral,yaw=self.speeds
        if forward>self.high[0] or backward>-self.low[0] or lateral>min(-self.low[1],self.high[1]) or yaw>min(-self.low[2],self.high[2]):
            raise ValueError('Keyboard speed exceeds verified policy range')
        self.accel=self._limits(config['acceleration_limit'])
        self.decel=self._limits(config['deceleration_limit'])
        self.pressed=set()
        self.held_keys=set()
        self.blocked=set()
        self.command=[0.0,0.0,0.0]
        self.focused=False
        self.running=False
        self.enabled=config.get('capture_keys_on_start',True)
        self.stop_down=False
        self.reset_requested=False
        self.reason='Waiting for viewport focus'
        self.event_count=0
        self.stop_generation=0
        self.follow_generation=0

    @staticmethod
    def _limits(value):
        values=[float(value)]*3 if isinstance(value,(float,int)) else [float(v) for v in value]
        if len(values)!=3 or not all(math.isfinite(v) and v>0 for v in values): raise ValueError('Invalid acceleration/deceleration limits')
        return values

    def clear(self,reason):
        self.blocked.update(self.pressed)
        self.pressed.clear()
        self.held_keys.clear()
        self.command=[0.0,0.0,0.0]
        self.reason=reason

    def set_focus(self,focused):
        focused=bool(focused)
        if focused!=self.focused:
            self.clear('Ready: press keys again' if focused else 'Focus lost: command zero')
            self.focused=focused

    def set_running(self,running):
        running=bool(running)
        if running!=self.running:
            self.clear('Resumed: fresh key press required' if running else 'Paused/stopped: command zero')
            self.running=running

    def set_enabled(self,enabled):
        self.enabled=bool(enabled)
        self.clear('Keyboard capture enabled' if self.enabled else 'Keyboard capture disabled')

    def request_reset(self):
        self.clear('Reset requested: waiting for safe state')
        self.reset_requested=True

    def handle_key(self,key,event_type):
        """event_type is PRESS/REPEAT/RELEASE; repeats never add increments."""
        if key not in self.mapping: return False
        self.event_count+=1
        action=self.mapping[key]
        if event_type=='RELEASE':
            self.pressed.discard(key)
            self.held_keys.discard(key)
            self.blocked.discard(key)
            if action=='stop': self.stop_down=False
            return True
        if event_type!='PRESS': return bool(self.enabled and self.focused)
        if not (self.enabled and self.focused): return False
        if action=='follow_camera':
            self.follow_generation+=1
            return True
        if action=='stop':
            self.stop_generation+=1
            self.clear('Stop: target command zero')
            self.stop_down=True
            self.held_keys.add(key)
            return True
        if action=='reset':
            self.request_reset()
            self.held_keys.add(key)
            return True
        if not self.running or self.stop_down or self.reset_requested:
            self.blocked.add(key)
            return True
        if key not in self.blocked:
            self.pressed.add(key)
            self.held_keys.add(key)
            self.reason='Keyboard command active'
        return True

    def desired(self):
        if not (self.enabled and self.focused and self.running) or self.stop_down or self.reset_requested: return [0.0]*3
        actions={self.mapping[k] for k in self.pressed}
        forward,backward,lateral,yaw=self.speeds
        def opposed(positive,negative,p,n):
            if positive in actions and negative in actions: return 0.0
            return p if positive in actions else (-n if negative in actions else 0.0)
        return [opposed('forward','backward',forward,backward),opposed('lateral_left','lateral_right',lateral,lateral),opposed('turn_left','turn_right',yaw,yaw)]

    def reconcile_released_keys(self,is_down,logger=None):
        """Release-only native safety polling; never synthesizes a movement press.

        Lost key-up callbacks must not leave W latched and cancel S/Down.
        Real simultaneous opposite keys retain their documented cancellation.
        """
        candidates=self.pressed | self.held_keys | self.blocked
        for key in list(candidates):
            if not is_down(key):
                self.handle_key(key,'RELEASE')
                if logger: logger({'key':key,'type':'RELEASE','source':'physical_release_reconcile','desired':self.desired()})

    def tick(self,dt):
        if not math.isfinite(dt) or dt<0: raise ValueError('Invalid control dt')
        desired=self.desired()
        if not (self.enabled and self.focused and self.running) or self.stop_down or self.reset_requested:
            self.command=[0.0]*3
            return list(self.command)
        for i,(current,target) in enumerate(zip(self.command,desired)):
            actions={self.mapping[k] for k in self.pressed}
            opposing=[('forward','backward'),('lateral_left','lateral_right'),('turn_left','turn_right')][i]
            if all(action in actions for action in opposing):
                self.command[i]=0.0
                continue
            # On reversal decelerate to zero first, then accelerate opposite.
            reversing=current*target<0
            intermediate=0.0 if reversing else target
            limit=self.decel[i] if reversing or abs(intermediate)<abs(current) else self.accel[i]
            delta=max(-limit*dt,min(limit*dt,intermediate-current))
            self.command[i]=max(self.low[i],min(self.high[i],current+delta))
        return list(self.command)

class CarbKeyboardInput:
    """Kit 110 ordered input capture; mapped keys stop viewport navigation.

    Optional logical keyboard/focus provider is dependency injection for tests.
    Production defaults to the app's native keyboard and focus state.
    """
    _owners={}
    def __init__(self,controller,keyboard=None,focus_provider=None,event_logger=None):
        import carb.input,omni.appwindow,omni.timeline
        self.carb=carb.input
        self.controller=controller
        self.appwindow=omni.appwindow.get_default_app_window()
        self.keyboard=keyboard if keyboard is not None else self.appwindow.get_keyboard()
        self.input=carb.input.acquire_input_interface()
        self.focus_provider=focus_provider or self.appwindow.is_focused
        self.logger=event_logger
        self.subscription=None
        self.observers=[]
        self.native=keyboard is None
        self.physical_key_down=None
        if self.native:
            import sys
            if sys.platform=='win32':
                import ctypes
                user32=ctypes.WinDLL('user32',use_last_error=True)
                query=user32.GetAsyncKeyState
                query.argtypes=[ctypes.c_int]; query.restype=ctypes.c_short
                codes={'UP':0x26,'DOWN':0x28,'LEFT':0x25,'RIGHT':0x27,'SPACE':0x20}
                self.physical_key_down=lambda key:bool(query(codes.get(key,ord(key) if len(key)==1 else 0)) & 0x8000)
        self.unknown_input_reported=False
        self.start()

    def start(self):
        if self.subscription is not None: return False
        import carb.eventdispatcher,omni.appwindow,omni.timeline
        key=hash(self.keyboard)
        old=self._owners.get(key)
        if old is not None and old is not self: old.close()
        self._owners[key]=self
        # Kit may route a native window keyboard through a different logical
        # device after docking/focus changes. Native capture listens to keyboard
        # events globally within this app, gated by the viewport/HUD focus.
        device=None if self.native else self.keyboard
        self.subscription=self.input.subscribe_to_input_events(self._on_input,device=device,order=self.controller.config['input_subscription_order'])
        dispatcher=carb.eventdispatcher.get_eventdispatcher()
        if self.native:
            self.observers.append(dispatcher.observe_event(event_name=omni.appwindow.GLOBAL_EVENT_WINDOW_FOCUS,filter=self.appwindow.get_event_key(),on_event=self._on_focus,observer_name='G1Keyboard.Focus'))
        for name,running in [(omni.timeline.GLOBAL_EVENT_PAUSE,False),(omni.timeline.GLOBAL_EVENT_STOP,False),(omni.timeline.GLOBAL_EVENT_PLAY,True)]:
            self.observers.append(dispatcher.observe_event(event_name=name,on_event=lambda e,value=running:self.controller.set_running(value),observer_name='G1Keyboard.Timeline.'+str(running)+name))
        self.refresh_focus()
        self.controller.set_running(omni.timeline.get_timeline_interface().is_playing())
        return True

    def _on_focus(self,event):
        if not event['isFocused']: self.controller.set_focus(False)
        else: self.refresh_focus()

    def refresh_focus(self):
        self.controller.set_focus(bool(self.focus_provider()))
        if self.native and self.physical_key_down is not None:
            self.controller.reconcile_released_keys(self.physical_key_down,self.logger)

    def _on_input(self,event):
        if event.deviceType!=self.carb.DeviceType.KEYBOARD: return True
        raw=event.event
        types={self.carb.KeyboardEventType.KEY_PRESS:'PRESS',self.carb.KeyboardEventType.KEY_RELEASE:'RELEASE',self.carb.KeyboardEventType.KEY_REPEAT:'REPEAT'}
        kind=types.get(raw.type)
        if kind is None:
            kind={'KEY_PRESS':'PRESS','KEY_RELEASE':'RELEASE','KEY_REPEAT':'REPEAT'}.get(getattr(raw.type,'name',str(raw.type)).split('.')[-1])
        if kind is None: return True
        key=raw.input.name if hasattr(raw.input,'name') else str(raw.input).upper()
        if key=='UNKNOWN':
            if self.native and self.logger and not self.unknown_input_reported:
                self.unknown_input_reported=True
                self.logger({'input_warning':'UNKNOWN native key; ignored without guessing a movement command'})
            return True
        self.refresh_focus()
        modifiers=self.carb.KEYBOARD_MODIFIER_FLAG_CONTROL|self.carb.KEYBOARD_MODIFIER_FLAG_ALT|self.carb.KEYBOARD_MODIFIER_FLAG_SUPER
        if raw.modifiers & modifiers and kind!='RELEASE':
            self.controller.clear('Modifier shortcut: command zero')
            return True
        handled=self.controller.handle_key(key,kind)
        if self.logger and key in self.controller.mapping:
            self.logger({'key':key,'type':kind,'handled':handled,'pressed':sorted(self.controller.pressed),'desired':self.controller.desired(),
                         'focused':self.controller.focused,'running':self.controller.running,'native_device':self.native})
        return not handled

    def close(self):
        if self.subscription is not None:
            self.input.unsubscribe_to_input_events(self.subscription)
            self.subscription=None
        self.observers.clear()
        key=hash(self.keyboard)
        if self._owners.get(key) is self: self._owners.pop(key,None)
        self.controller.clear('Keyboard subscription closed')
