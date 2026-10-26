"""Collision workaround must preserve tier height and circular boundary."""
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from pathlib import Path
import numpy as np
import trimesh
from xrobocon.field import XRoboconField

class FieldMeshTests(unittest.TestCase):
    def test_mesh_preserves_field_dimensions(self):
        class Scene:
            def __init__(self):self.morphs=[]
            def add_entity(self,morph,**kwargs):self.morphs.append(morph);return morph
        field=XRoboconField();scene=Scene()
        with tempfile.TemporaryDirectory() as folder, patch('xrobocon.field.gs') as genesis:
            genesis.morphs.Mesh.side_effect=lambda **kwargs:SimpleNamespace(**kwargs)
            field.build(scene,collision_mesh_dir=folder)
            self.assertEqual(len(scene.morphs),3)
            for i,(tier,morph) in enumerate(zip(field.tiers,scene.morphs)):
                mesh=trimesh.load(Path(folder)/f'tier-{i+1}.obj',force='mesh')
                world=mesh.vertices+np.array(morph.pos)
                self.assertAlmostEqual(world[:,2].min(),0,places=7)
                self.assertAlmostEqual(world[:,2].max(),tier['height'],places=7)
                rim=world[np.linalg.norm(world[:,:2],axis=1)>tier['radius']*.9,:2]
                self.assertLess(abs(np.linalg.norm(rim,axis=1).max()-tier['radius']),1e-7)
                # Chord midpoints give the worst radial deviation.
                angles=np.sort(np.arctan2(rim[:,1],rim[:,0]))
                largest_gap=np.diff(np.r_[angles,angles[0]+2*np.pi]).max()
                self.assertLess(tier['radius']*(1-np.cos(largest_gap/2)),.0004)
