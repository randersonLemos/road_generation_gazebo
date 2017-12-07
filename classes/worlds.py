import os
import others.utils as utils
import warnings
from frozen import FrozenClass



class World(FrozenClass):

  def __init__( self, world_file=None, model=None
               ,poses=None ,fname='custom.world'):

    try: # loading word
      with open(world_file, 'r') as file_handle:
        self.world = file_handle.read()
    except:
      self.world = utils.EMPTY_WORLD
      warnings.warn( '.world file not specified.'
                    +' Using world defined in variable'
                    +' utils.EMPTY_WORLD...')

    if model:
      self.model = model
    else:
      self.model = 'cylinder_custom'
      warnings.warn( 'model not specified.'
                    +' Using the model cylinder_custom'
                    +'available at folder models')

    if poses:
      self.poses = poses
    else:
      self.poses=[
                   [(1.0),(0.0),(0.0),(0.0),(0.0),(0.0)]
                  ,[(2.0),(0.0),(0.0),(0.0),(0.0),(0.0)]
                  ,[(3.0),(0.0),(0.0),(0.0),(0.0),(0.0)]
                 ]
      warnings.warn( 'poses not specified.'
                    +' Using the default:'
                    +' ' +  str(self.poses))

    self.fname = fname

  def make_custom_world(self, save=True):
    stg = '</include>\n'
    idx = self.world.rfind(stg)
    lst = []
    for pose in self.poses:
      lst.append(utils.include_model(self.model, pose))
    to_inset =  '\n'.join(lst)
    self.custom_world =  self.world[:idx+len(stg)] \
                        +to_inset \
                        +self.world[idx+len(stg)-1:]
    if save:
      self._save_custom_world()

  def _save_custom_world(self):
    if not os.path.exists('worlds'):
      os.makedirs('worlds')

    with open('worlds/'+self.fname, 'w') as file_handle:
      file_handle.write(self.custom_world)
