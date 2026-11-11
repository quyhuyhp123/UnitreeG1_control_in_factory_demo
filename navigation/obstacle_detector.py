"""USD/PhysX ground truth, explicitly not AI or sensor perception."""
import math


class SensorPerception:
    mode='SENSOR (not implemented)'
    def update(self,*args,**kwargs): raise NotImplementedError('No LiDAR/depth/tracker enabled')


class SceneObstacleDetector:
    mode='SIM GROUND TRUTH'
    def __init__(self,stage,config,root='/World/FactoryScene'):
        self.stage=stage; self.config=config; self.root=root; self.previous={}; self.obstacles=[]; self.timestamp=None; self.floor=None

    def update(self,now):
        from pxr import Usd,UsdGeom,UsdPhysics
        cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy'],False,True)
        objects=[]; zmin,zmax=self.config['obstacle_height_range_m']
        for p in self.stage.Traverse():
            path=str(p.GetPath())
            if path.startswith('/World/G1') or path.startswith('/World/Navigation'): continue
            if not p.HasAPI(UsdPhysics.CollisionAPI): continue
            if not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get(): continue
            bounds=cache.ComputeWorldBound(p).ComputeAlignedRange()
            if bounds.IsEmpty(): continue
            lo=list(bounds.GetMin()); hi=list(bounds.GetMax())
            if p.GetName().startswith('Floor') and hi[2]<=.01:
                self.floor=[lo[:2],hi[:2]]; continue
            if hi[2]<=zmin or lo[2]>=zmax: continue
            kind='static'; dynamic=False; a=p; human_id=None
            while a and not a.IsPseudoRoot():
                name=a.GetName().lower()
                attr=a.GetAttribute('navigation:kind')
                if 'worker' in name or 'human' in name or (attr and attr.Get()=='human'):
                    kind='human'; human_id=str(a.GetPath())
                if a.HasAPI(UsdPhysics.RigidBodyAPI): dynamic=True
                a=a.GetParent()
            center=[(lo[i]+hi[i])/2 for i in [0,1]]; velocity=[0.,0.]
            previous=self.previous.get(path)
            if previous and now>previous[0]: velocity=[(center[i]-previous[1][i])/(now-previous[0]) for i in [0,1]]
            if math.hypot(*velocity)>.01: dynamic=True
            self.previous[path]=(now,center)
            objects.append({'id':path,'kind':kind,'human_id':human_id,'dynamic':dynamic,'bounds':[lo[:2],hi[:2]],
                            'height_range_m':[lo[2],hi[2]],'velocity':velocity,'timestamp':now})
        self.obstacles=objects; self.timestamp=now
        return objects
