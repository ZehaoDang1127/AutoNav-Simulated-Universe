"""Guidance cannot outlive its local sensor observations or frame."""
import unittest
from autonav_universe.guidance_cache import GuidanceCache
from autonav_universe.corridor_tangent import TangentEstimate


class GuidanceCacheTests(unittest.TestCase):
    def setUp(self):
        self.c=GuidanceCache(.75)
        self.e=TangentEstimate(1.0,'divider',.95,12)
        self.c.update(self.e,'odom',10.0)
    def test_fresh_guidance_is_available(self):
        self.assertIs(self.c.get('odom',10.5,{'divider':True}),self.e)
        self.assertEqual(self.c.reference(.4,'odom',10.5,{'divider':True}),1.0)
    def test_stale_time_source_or_frame_never_orients_new_data(self):
        for frame,now,fresh in [('odom',10.76,{'divider':True}),('map',10.5,{'divider':True}),
                                ('odom',10.5,{'divider':False}),('odom',9.9,{'divider':True}),
                                ('odom',float('nan'),{'divider':True})]:
            self.assertIsNone(self.c.get(frame,now,fresh))
            self.assertEqual(self.c.reference(.4,frame,now,fresh),.4)
    def test_boundary_freshness_is_independent_of_divider(self):
        self.c.update(TangentEstimate(.6,'boundary',.9,9),'odom',11)
        self.assertIsNone(self.c.get('odom',11,{'divider':True,'boundary':False}))
        self.assertEqual(self.c.get('odom',11,{'boundary':True}).yaw_rad,.6)
    def test_clear_and_validation(self):
        self.c.clear()
        self.assertIsNone(self.c.get('odom',10,{'divider':True}))
        with self.assertRaises(ValueError): self.c.update(self.e,'',10)
        with self.assertRaises(ValueError): GuidanceCache(0)


if __name__=='__main__':unittest.main()
