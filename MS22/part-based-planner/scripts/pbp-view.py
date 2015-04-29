#!/usr/bin/env python2.7

from pbp import *

if __name__ == "__main__":
    
    import argparse
    parser = argparse.ArgumentParser()

    # inputs
    parser.add_argument('-d', default=None)
    parser.add_argument('-r', action='store_true')
    parser.add_argument('-v', '--verbose', default=1, type=int)
    parser.add_argument('--config', default=None)

    args = parser.parse_args()

    dir = args.d
    show_rois = args.r
    verbose = args.verbose
    configFile = args.config

    pbp = PBP(configFile, verbose)
    if dir == None:
      dir = pbp.lastDir()
    plans = PlanCollection(pbp.wd, dir)
    plans.loadPlans()
    pbp.view(plans, show_rois)
