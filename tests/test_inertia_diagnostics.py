import logging,unittest
from xrobocon.inertia_diagnostics import InertiaFrameFilter
class InertiaDiagnosticsTests(unittest.TestCase):
    def test_only_proven_frame_mismatch_is_filtered(self):
        f=InertiaFrameFilter({'left_bogie':([.0018,.0018,.0000095],[.0000095,.0018,.0018])})
        def record(message):return logging.LogRecord('genesis',logging.WARNING,'',0,message,(),None)
        prefix="Link 'left_bogie' has dubious inertia [ixx=1.8e-3,iyy=1.8e-3,izz=9.5e-6] compared to the estimate from geometry "
        self.assertFalse(f.filter(record(prefix+'[ixx=9.5e-6,iyy=1.8e-3,izz=1.8e-3].')))
        self.assertTrue(f.filter(record(prefix+'[ixx=1,iyy=1,izz=1].')))
        self.assertTrue(f.filter(record('Other important warning')))
        self.assertEqual(f.corrected,1)
    def test_contact_exclusions_do_not_change_model_inertia(self):
        import xml.etree.ElementTree as ET
        import mujoco,numpy as np
        path='xrobocon/assets/robot_passive_front_bogie_r18_steering.xml'
        model=mujoco.MjModel.from_xml_path(path);root=ET.parse(path).getroot()
        self.assertEqual(len(root.find('contact').findall('exclude')),4)
        root.remove(root.find('contact'));baseline=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        for key in ('body_mass','body_inertia','body_iquat','body_pos','geom_pos','geom_quat','jnt_axis'):
            np.testing.assert_array_equal(getattr(model,key),getattr(baseline,key))
