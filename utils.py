import pickle

#--- read trajectory
def get_trajectory(traj_num):
  """
  The function `get_trajectory` loads a selected data file and returns the input and output trajectory
  sequences for a given trajectory number.
  
  :param traj_num: The index of the trajectory sequence to be retrieved from the loaded data
  :return: input and output trajectory sequences
  """
  try:
    with open("selected_data.pkl", "rb") as f: 
      load_data = pickle.load(f) # with batch size 6770
      # ['input_traj', 'output_traj', 'text', 'obj_names', 'obj_poses', 'obj_classes', 'obj_in_text', 'image_paths']
      # keys = list(load_data.keys())
    
    # pick the input and output trajectory sequences
    input_traj = load_data['input_traj'][traj_num]
    output_traj = load_data['output_traj'][traj_num]

    return input_traj, output_traj
  
  except:
    print("error: trajectory not loaded")

def load_gazebo_trajectory():
    """
    This function loads a Gazebo trajectory from a pickle file.
    :return: The function `load_gazebo_trajectory()` is returning the object `gazebo_traj`, which is
    loaded from the file "gazebo_trajectory.pkl" using the `pickle.load()` method.
    """

    with open("gazebo_trajectory.pkl", "rb") as f: 
        gazebo_traj = pickle.load(f) 

    return gazebo_traj


def save_trajectory(gazebo_traj):
    """
    This function saves a gazebo trajectory to a pickle file.
    
    :param gazebo_traj: The parameter `gazebo_traj` is a variable that contains the trajectory data that
    needs to be saved. It is likely a data structure that contains information about the position,
    velocity, and acceleration of a robot or object in a Gazebo simulation environment. The function
    `save_trajectory` takes this
    """
      # save trajectory once exiting spin loop
    with open('gazebo_trajectory.pkl','wb') as f:
      pickle.dump(gazebo_traj, f)