"""Author two static USD skeleton poses from the bundled worker rig."""
from pathlib import Path
import re
from pxr import Gf,Sdf,Usd,UsdSkel,Vt

ROOT=Path(__file__).resolve().parents[1]/'assets/factory/factory_layout_v2'
WORKER=ROOT/'props/worker'
SOURCE='male_adult_construction_05_new.usd'

def build(name,working):
    base=Usd.Stage.Open(str(WORKER/SOURCE))
    bone=next(p for p in base.Traverse() if p.IsA(UsdSkel.Skeleton))
    skeleton=UsdSkel.Skeleton(bone)
    joints=skeleton.GetJointsAttr().Get()
    rest=skeleton.GetRestTransformsAttr().Get()
    topology=UsdSkel.Topology(joints)
    parents=topology.GetParentIndices()
    names={str(j).rsplit('/',1)[-1]:i for i,j in enumerate(joints)}
    transforms=Vt.Matrix4dArray(rest)
    for side,sign in [('L',1),('R',-1)]:
        for part,child,direction in [
            ('Upperarm','Forearm',(sign*.14,-.38,-.91) if working else (sign*.09,0,-1)),
            ('Forearm','Hand',(-sign*.19,-.98,.12) if working else (sign*.06,-.08,-1)),
        ]:
            i=names[f'{side}_{part}']; j=names[f'{side}_{child}']
            world=UsdSkel.ConcatJointTransforms(topology,transforms)
            parent=world[parents[i]]
            current=world[j].ExtractTranslation()-world[i].ExtractTranslation()
            inverse=parent.GetInverse()
            rotation=Gf.Rotation(inverse.TransformDir(current).GetNormalized(),
                                 inverse.TransformDir(Gf.Vec3d(*direction)).GetNormalized())
            delta=Gf.Matrix4d(1).SetRotate(rotation)
            affected=[i]
            twist=names[f'{side}_UpperarmTwist01']
            if part=='Upperarm' and parents[twist]==parents[i]:affected.append(twist)
            for index in affected:
                local=transforms[index]
                transforms[index]=(local*delta).SetTranslateOnly(local.ExtractTranslation())
    layer=Sdf.Layer.CreateAnonymous('worker_pose.usda')
    # Compose against the original via its absolute path while authoring, then
    # export with a relative dependency so Linux and Windows can both load it.
    layer.subLayerPaths=[str(WORKER/SOURCE)]
    stage=Usd.Stage.Open(layer)
    stage.SetDefaultPrim(stage.GetPrimAtPath('/Root'))
    animation=UsdSkel.Animation.Define(stage,'/Root/StaticWorkerPose')
    animation.CreateJointsAttr(joints)
    t,r,s=UsdSkel.DecomposeTransforms(transforms)
    animation.CreateTranslationsAttr(t);animation.CreateRotationsAttr(r);animation.CreateScalesAttr(s)
    UsdSkel.BindingAPI.Apply(stage.GetPrimAtPath(bone.GetPath())).CreateAnimationSourceRel().SetTargets([animation.GetPath()])
    output=Sdf.Layer.FindOrOpen(str(WORKER/f'{name}.usda')) or Sdf.Layer.CreateNew(str(WORKER/f'{name}.usda'))
    output.TransferContent(layer)
    output.subLayerPaths=['./'+SOURCE]
    output.Save()

if __name__=='__main__':
    build('standing',False)
    build('working',True)
    for path in [ROOT/'factory_layout_v2_workers.usda',ROOT/'maps/map_2.usda',ROOT/'maps/map_3.usda']:
        pose='working' if path.parent==ROOT else 'standing'
        source=path.read_text(encoding='utf-8')
        updated=re.sub(r'(props/worker/)(?:male_adult_construction_05_new\.usd|standing\.usda|working\.usda)',rf'\g<1>{pose}.usda',source)
        if source!=updated:path.write_text(updated,encoding='utf-8')
