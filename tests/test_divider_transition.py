"""Lane change costs require actual observed side-to-side transitions."""
import math
import unittest
from autonav_universe.local_drive import LocalGrid
from autonav_universe.divider_transition import observed_divider_transitions


def divider(resolution=.1, angle=0):
    width=height=round(10/resolution)
    cells=tuple(100 if abs((iy+.5)*resolution-5)<.11 else 0
                for iy in range(height) for _ in range(width))
    return LocalGrid(width,height,resolution,-5,-5,angle,cells)


def world(p, origin, angle):
    c,s=math.cos(angle),math.sin(angle)
    return origin[0]+c*p[0]-s*p[1],origin[1]+s*p[0]+c*p[1]


class DividerTransitionTests(unittest.TestCase):
    def test_side_to_side_and_back_count_once_each(self):
        for res in (.05,.1,.2):
            grid=divider(res)
            self.assertEqual(observed_divider_transitions([(0,-1),(0,1)],grid),1)
            self.assertEqual(observed_divider_transitions([(0,-1),(0,1),(0,-1)],grid),2)
            dense=[(0,-.9+i*.025) for i in range(73)]
            self.assertEqual(observed_divider_transitions(dense,grid),1)

    def test_touch_or_partial_entry_is_not_a_lane_change(self):
        grid=divider()
        self.assertEqual(observed_divider_transitions([(0,-1),(0,0),(1,-1)],grid),0)
        self.assertEqual(observed_divider_transitions([(0,-1),(0,0)],grid),0)
        self.assertEqual(observed_divider_transitions([(0,0),(0,1)],grid),0)
        self.assertEqual(observed_divider_transitions([(0,-1),(3,-1)],grid),0)

    def test_only_supported_divider_is_counted(self):
        grid=divider()
        empty=LocalGrid(grid.width,grid.height,grid.resolution,grid.origin_x,grid.origin_y,0,(0,)*len(grid.cells))
        self.assertEqual(observed_divider_transitions([(0,-1),(0,1)],empty),0)
        self.assertEqual(observed_divider_transitions([(0,-1),(0,1)],None),0)

    def test_rotated_costmap(self):
        angle=.42
        grid=divider(angle=angle)
        before=world((5,4),(-5,-5),angle)
        after=world((5,6),(-5,-5),angle)
        self.assertEqual(observed_divider_transitions([before,after],grid),1)
        self.assertEqual(observed_divider_transitions([before,world((5,5),(-5,-5),angle),before],grid),0)

    def test_single_unclassified_point_cannot_define_sides(self):
        grid=divider(); cells=[0]*len(grid.cells)
        cells[50*grid.width+50]=100
        mask=LocalGrid(grid.width,grid.height,grid.resolution,grid.origin_x,grid.origin_y,0,tuple(cells))
        self.assertEqual(observed_divider_transitions([(.05,-.2),(.05,.2)],mask),0)


if __name__=='__main__': unittest.main()
