"""Observed lane geometry tests; no authored track points reach the estimator."""
import math
import unittest
from autonav_universe.corridor_tangent import estimate_corridor_tangent
from autonav_universe.local_drive import LocalGrid


def make_grid(predicate, yaw=0.0, resolution=.1):
    width = height = 120
    ox, oy = -6.0, -6.0
    c,s = math.cos(yaw),math.sin(yaw)
    cells=[]
    for iy in range(height):
        for ix in range(width):
            a, b=(ix+.5)*resolution,(iy+.5)*resolution
            x,y=ox+c*a-s*b,oy+s*a+c*b
            cells.append(100 if predicate(x,y) else 0)
    return LocalGrid(width,height,resolution,ox,oy,yaw,tuple(cells))


class CorridorTangentTests(unittest.TestCase):
    def test_forward_and_reverse_use_observed_same_stripe(self):
        grid=make_grid(lambda x,y: abs(y-1.2)<.06)
        f=estimate_corridor_tangent(grid,0,0,0)
        b=estimate_corridor_tangent(grid,0,0,math.pi)
        self.assertIsNotNone(f); self.assertIsNotNone(b)
        self.assertLess(abs(f.yaw_rad),.1)
        self.assertLess(abs(abs(b.yaw_rad)-math.pi),.1)
        self.assertEqual(f.source,'divider')
        self.assertGreaterEqual(f.confidence,.65)

    def test_progressively_curving_line_tracks_direction(self):
        # Tangent of y=0.2*x^2 at the lookahead x=1 is atan(.4).
        grid=make_grid(lambda x,y: abs(y-(1+.2*x*x))<.06 and -1<x<3)
        v=estimate_corridor_tangent(grid,0,0,0)
        self.assertIsNotNone(v)
        self.assertGreater(v.yaw_rad,.1)
        self.assertLess(v.yaw_rad,.8)

    def test_sparse_distant_and_unclassified_data_give_none(self):
        empty=make_grid(lambda x,y: False)
        tiny=make_grid(lambda x,y: abs(x-1)<.1 and abs(y-1)<.1)
        far=make_grid(lambda x,y: abs(y-5)<.06)
        self.assertIsNone(estimate_corridor_tangent(None,0,0,0))
        self.assertIsNone(estimate_corridor_tangent(empty,0,0,0))
        self.assertIsNone(estimate_corridor_tangent(tiny,0,0,0))
        self.assertIsNone(estimate_corridor_tangent(far,0,0,0))

    def test_perpendicular_line_does_not_invent_turn_direction(self):
        grid=make_grid(lambda x,y: abs(x-1)<.06 and abs(y)<2)
        self.assertIsNone(estimate_corridor_tangent(grid,0,0,0))

    def test_ambiguous_isotropic_pattern_rejected(self):
        grid=make_grid(lambda x,y: math.hypot(x-1,y)<.85)
        self.assertIsNone(estimate_corridor_tangent(grid,0,0,0))

    def test_line_only_fallback_and_rotated_grid(self):
        empty=make_grid(lambda x,y: False)
        line=make_grid(lambda x,y: abs(y-1)<.05, yaw=.2)
        result=estimate_corridor_tangent(empty,0,0,0,boundary=line)
        self.assertIsNotNone(result)
        self.assertEqual(result.source,'boundary')
        self.assertLess(abs(result.yaw_rad),.12)

    def test_boundary_fit_ignores_inflation_halo(self):
        line=make_grid(lambda x,y: abs(y-1)<.06)
        values=list(line.cells)
        for i, value in enumerate(values):
            wx=line.origin_x+(i%line.width+.5)*line.resolution
            wy=line.origin_y+(i//line.width+.5)*line.resolution
            if not value and abs(wy-1)<1.1 and -1<wx<3:
                values[i]=65
        inflated=LocalGrid(line.width,line.height,line.resolution,line.origin_x,
                           line.origin_y,line.origin_yaw,tuple(values))
        result=estimate_corridor_tangent(None,0,0,0,boundary=inflated)
        self.assertIsNotNone(result)
        self.assertEqual(result.source,'boundary')
        self.assertGreater(result.confidence,.95)
        self.assertLess(abs(result.yaw_rad),.1)
        halo_only=LocalGrid(line.width,line.height,line.resolution,line.origin_x,
                            line.origin_y,line.origin_yaw,tuple(min(v,99) for v in values))
        self.assertIsNone(estimate_corridor_tangent(None,0,0,0,boundary=halo_only))

    def test_invalid_settings_fail(self):
        with self.assertRaises(ValueError):
            estimate_corridor_tangent(None,0,0,0,fit_radius_m=0)


if __name__=='__main__': unittest.main()
