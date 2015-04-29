#!/usr/bin/env python2.7

import shlex
import subprocess
import os.path
import os
from os.path import join
import ConfigParser
from operator import itemgetter
from multiprocessing import Process, Queue

def term_red(s):
    return '\033[0;31m' + str(s) + '\033[0m'
def term_green(s):
    return '\033[0;32m' + str(s) + '\033[0m'
def term_blue(s):
    return '\033[0;34m' + str(s) + '\033[0m'
def term_plain(s):
    return str(s)

if 'TERM' in os.environ and os.environ['TERM'] != 'dumb':
    pass
else:
    term_red = term_plain
    term_green = term_plain
    term_blue = term_plain

def system(cmd, verbose, nocheck = False):
    if verbose:
      print cmd
    a = shlex.split(cmd)
    if nocheck:
        return subprocess.call(a)
    else:
        return subprocess.check_call(a)

def system_ret(cmd, verbose, nocheck = False):
    if verbose:
      print cmd
    a = shlex.split(cmd)
    po = subprocess.Popen(a, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    rv = po.communicate()
    rc = po.returncode
    if rc == 0 or nocheck:
        return rv[0]
    else:
        print rv[0] + "\n" + rv[1]
        raise subprocess.CalledProcessError(rc, a, rv[0] + "\n" + rv[1])

class Plan(object):
  def __init__(self, proto, score, nPoints, pose, preshape):
    self.proto = proto
    self.score = score
    self.nPoints = nPoints
    self.pose = pose
    self.preshape = preshape

class PlanCollection(object):
  def __init__(self, datadir, planName):
    self._datadir = datadir
    self._plandir = join(datadir, planName)
    self.plans = []
  def add(self, plan):
    self.plans.append(plan)
    self.plans.sort(key=lambda e: e.score, reverse=True)
  def loadPlans(self):
    c = ConfigParser.ConfigParser()
    c.readfp(open(join(self._plandir, "plans.cfg"), 'r'))
    nProto = c.getint("plancollection", "n_prototypes")
    for i in range(nProto):
      p = Plan(i,
               c.getfloat(str(i), "score"),
               c.getint(str(i), "n_points"),
               [float(e) for e in c.get(str(i), "pose").strip().split()],
               c.get(str(i), "preshape"))
      self.add(p)
  def savePlans(self):
    c = ConfigParser.ConfigParser()
    c.add_section("plancollection")
    c.set("plancollection", "n_prototypes", len(self.plans))
    for p in self.plans:
      c.add_section(str(p.proto))
      c.set(str(p.proto), "score", p.score)
      c.set(str(p.proto), "n_points", p.nPoints)
      c.set(str(p.proto), "pose", " ".join([str(k) for k in p.pose]))
      c.set(str(p.proto), "preshape", p.preshape)
    c.write(open(join(self._plandir, "plans.cfg"), 'w'))
  def saveLegacyPlans(self, filename = "scores.txt"):
    with open(join(self._plandir, filename), 'w') as f:
      for p in self.plans:
        f.write(str(p.proto) + " " + str(p.score) + " " + str(p.nPoints) + " " + " ".join([str(k) for k in p.pose]) + "\n")
  def dataPath(self, p):
    return join(self._datadir, p)
  def planPath(self, p):
    return join(self._plandir, p)

class PBP(object):

  def __init__(self, configFile, verbose = 1):    
      
    self.verbose = verbose
    self.config = ConfigParser.ConfigParser()
    if configFile:
      self.config.readfp(open(configFile))
    self.nChains = 6
    
    self.wd = os.getcwd()
    
    self.configSection = "pbp-config"
    self.inputName = self.param("input_name", 'image-0.pcd')
    self.autodetectInputType = self.param("autodetect_input_type", False)
    self.inputType = self.param("input_type", 'pcd')
    self.crop = self.param("crop_pcd_file", True)
    self.cropboxInKinectCoordinates = self.param("cropbox_file", "workspace.inkinect.m.txt")
    self.scale = self.param("scale", 1000.)
    self.removePlane = self.param("remove_dominant_plane", True)
    self.fitPlane = self.param("fit_plane", True)
    self.planeInlierThreshold = self.param("plane_inlier_threshold", 8.)
    self.transform = self.param("transform", True)
    self.kinectToRobot = self.param("transformation_file", "from-kinect-to-robot.txt")
    self.extraObstacles = self.param("extra_obstacles", True)
    self.extraObstaclesFile = self.param("extra_obstacles_file", "obst.pcd")
    self.nPrototypes = self.param("n_prototypes", 5)
    self.minDist = self.param("min_dist", 3.)
    self.locH = self.param("pe_position_bandwidth", 10.)
    self.partialviewPE = self.param("partial_view_pe", False)
    self.cif = self.param("custom_integrand_factor", "")
    self.meshVisibility = self.param("point_to_mesh_visibility_dist", 4.)
    self.handApproachAxis = self.param("hand_approach_axis", 0)

  def param(self, option, defValue):
    if self.config.has_option(self.configSection, option):
      if type(defValue) is int:
        o = self.config.getint(self.configSection, option)
      elif type(defValue) is float:
        o = self.config.getfloat(self.configSection, option)
      elif type(defValue) is bool:
        o = self.config.getboolean(self.configSection, option)
      else:
        o = self.config.get(self.configSection, option)
    else:
      o = defValue
    return o

  def fparam(self, option, defValue):
    return os.path.abspath(self.param(option, defValue))

  def system(self, cmd, nocheck = False):
    if self.verbose == 0:
      return system_ret(cmd, False, nocheck)
    elif self.verbose == 1:
      return system_ret(cmd, False, nocheck)
    else:
      return system(cmd, True, nocheck)

  def system_ret(self, cmd, nocheck = False):
    if self.verbose < 2:
      return system_ret(cmd, False, nocheck)
    else:
      return system_ret(cmd, True, nocheck)

  def log(self, message):
    if self.verbose > 0:
      print message

  def nextFreeDir(self, dir = None):
    os.chdir(self.wd)
    if dir == None:
      i = 0
    else:
      i = int(dir)      
    while os.path.exists(str(i)):
      i = i+1
    return str(i)

  def lastDir(self):
    i = 0
    dir = None
    while os.path.exists(str(i)):
      dir = str(i)
      i = i+1
    if dir == None:
      raise Error("No directory exists")
    else:
      return dir

  def pe(self, i, r):
      extraArg = ""
      try:
        backoff = open("../prototypes/" + str(i) + ".backoff", 'r').readline().strip()
      except:
        backoff = "100"
      
      if self.partialviewPE:
        extraArg += " --partial --viewpoint " + join(self.wd, self.kinectToRobot) + " "  + ' --point_to_mesh_visibility_dist ' + str(self.meshVisibility) + ' '
      if self.cif and self.cif != "":
        extraArg +=  ' --cif ' + self.cif + ' '
      if self.fitPlane:
        extraArg += ' --table image-0.plane '
      self.system('pbp-plan -c ' + str(self.nChains) + ' -l ' + str(self.locH) + ' -o 0.1 --arm-nr 1 --bboxC part.bbox --bboxT ../prototypes/' + str(i) + '.bbox --normals  ../prototypes/' + str(i) + '.xml image-0.crop -s --aligned aligned-' + str(i) + '.xml --log log-' + str(i) + '.ini --best_transfo t-' + str(i) + ' --obst image-0.obst --rois ../prototypes/' + str(i) + '.roi ' + ' ' + extraArg + ' --backoff_dist ' + str(backoff) + ' --hand_approach_axis ' + str(self.handApproachAxis))
      #self.log('pbp-plan -c ' + str(self.nChains) + ' -l ' + str(self.locH) + ' -o 0.1 --normals  ../prototypes/' + str(i) + '.xml image-0.crop -s --aligned aligned-' + str(i) + '.xml --log log-' + str(i) + '.ini --best_transfo t-' + str(i) + ' --obst image-0.obst --rois ../prototypes/' + str(i) + '.roi ' + ' ' + extraArg + ' --backoff_dist ' + str(backoff) + ' --hand_approach_axis ' + str(self.handApproachAxis))
      config = ConfigParser.ConfigParser()
      config.read('log-' + str(i) + '.ini')
      score = config.getfloat("pbp", "score")
      np = config.getint("pbp", "object_model_n_points")
      pose = config.get("pbp", "pose")
      preshape = open("../prototypes/" + str(i) + ".preshape", 'r').readline().strip()
      r.put(Plan(i, score, np, [float(e) for e in pose.strip().split()], preshape))

  def plan(self, dir):
    
    if not os.path.exists(dir):
      os.mkdir(dir)
    os.chdir(dir)
  
    try:
      if not os.path.exists(self.inputName):
          self.log("Capturing image...")
          self.system('kinect-capture')
      else:
          self.log("Importing existing image...")
    
      import tempfile
      tmp = tempfile.mkstemp()[1]

      self.log("Preprocessing point cloud...")

      input_type_arg = ''
      if not self.autodetectInputType:
        input_type_arg = ' -r ' + self.inputType + ' '

      if self.crop:
        self.system('nuklei conv --box_roi "{roi}" {ita} -w txt {input} {tmp}'.format(roi = open(join(self.wd, self.cropboxInKinectCoordinates)).readline().strip(), tmp=tmp, input=self.inputName, ita=input_type_arg))
      else:
        self.system('nuklei conv {ita} -w txt {input} {tmp}'.format(tmp=tmp, input=self.inputName, ita=input_type_arg))

      #self.system('nuklei conv --min_dist .009 {tmp} {tmp}')
      if abs(self.scale-1) > 1e-12:
        self.system('nuklei conv --scale {scale} -r txt {tmp} {tmp}'.format(tmp=tmp,scale=self.scale))
  
      if self.transform:
        self.system('nuklei conv --transformation {k} -r txt {tmp} {tmp}'.format(tmp=tmp, k=join(self.wd, self.kinectToRobot)))
  
      if self.removePlane or self.fitPlane:
        arg = ''
        if self.removePlane:
          arg += " --remove_dominant_plane"
        if self.fitPlane:
          arg += " --fitted_plane image-0.plane"
        self.system('nuklei conv ' + arg + ' --inlier_threshold {planeInlierThreshold} -r txt {tmp} {tmp}'.format(tmp=tmp,planeInlierThreshold=self.planeInlierThreshold))
  
      self.system('nuklei conv --set_rgb "1 0 0" -r txt {tmp} image-0.crop -w serial'.format(tmp=tmp))
      self.system('nuklei conv --set_rgb "1 0 0" -r txt {tmp} --min_dist {min_dist} image-0.crop.light -w serial'.format(tmp=tmp,min_dist=self.minDist))
  
      if self.extraObstacles:
        self.system('nuklei conv  --uniformize_weights {obst} image-0.crop.light -w txt image-0.obst'.format(tmp=tmp, obst=join(self.wd, self.extraObstaclesFile)))
      else:
        self.system('nuklei conv  --uniformize_weights image-0.crop.light -w txt image-0.obst'.format(tmp=tmp, obst=join(self.wd, self.extraObstaclesFile)))

      r = Queue()
      jobs = []
      for i in range(self.nPrototypes):
          self.log("Planning for prototype " + str(i) + "...")
          p = Process(target=self.pe, args=(i, r, ))
          jobs.append(p)
          p.start()
      for p in jobs:
          p.join()
      
      plans = PlanCollection(self.wd, dir)
      while not r.empty():
          plans.add(r.get())
      plans.savePlans()
      # to support legacy code:
      plans.saveLegacyPlans()
      
      #self.log("Prototype " + str(s[0]) + " matching with score " + str(s[1]) + ".")

      os.chdir(self.wd)
      return plans
    except:
      os.chdir(self.wd)
      raise
    
  def printPlans(self, plans, proto = None, order = None):
    for i, p in enumerate(plans.plans):
      if proto != None and p.proto != proto:
        continue
      if order != None and i != order:
        continue
      pose = "{:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}".format(*p.pose)
      self.log("Prototype " + str(p.proto) + ": score=" + str(p.score) + ", preshape=" + p.preshape)
      self.log("Pose: " + pose)
      
  def view(self, plans, show_rois = False, proto = None, order = None):

    self.printPlans(plans, proto = proto, order = order)
  
    for i, p in enumerate(plans.plans):
      if proto != None and p.proto != proto:
        continue
      if order != None and i != order:
        continue
      self.log("Displaying prototype " + str(p.proto) + ".")
      type = open(plans.dataPath('prototypes/' + str(p.proto) + '.preshape')).readline().strip()
      if type == "hcpower":
          type = "parallel"
      elif type == "hcpinch":
          type = "pinch"

      name = os.getcwd()
      cmd = 'yawan'
      cmd += ' -t ' + type
      cmd += ' -w ' + name
      cmd += ' -p ' + plans.dataPath('default_camera_pose.txt')
      cmd += ' ' + plans.planPath('aligned-' + str(p.proto) + '.xml')
      cmd += ' ' + plans.planPath('image-0.crop.light')
      cmd += ' ' + plans.dataPath('scene/part.pcd') # current part in green
      if self.extraObstacles:
        if not os.path.exists(join(self.wd, 'obst.light.txt')):
          self.system('nuklei conv {obst} -w txt -n 1000 {obstlight}'.format(obst=join(self.wd, self.extraObstaclesFile), obstlight=join(self.wd, 'obst.light.txt')))
        cmd += ' ' + plans.dataPath('obst.light.txt')
      #cmd += ' ' + plans.dataPath(self.kinectToRobot) #vis kinect to robot
     # cmd += ' --mesh_model ' + plans.dataPath('prototypes/' + str(p.proto) + '.ply') + ',' + plans.planPath('t-' + str(p.proto))
      #cmd += ' ' + plans.dataPath('gripper-' + type) + ',' + plans.planPath('t-' + str(p.proto))
      if show_rois:
        cmd += ' -b ' + plans.dataPath('prototypes/' + str(p.proto) + '.roi') + ',' + plans.planPath('t-' + str(p.proto))
      self.system(cmd)


if __name__ == "__main__":
  print "Base scripts for part-based grasping."
