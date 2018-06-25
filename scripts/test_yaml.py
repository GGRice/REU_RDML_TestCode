#!/usr/bin/env python

import rospy
import yaml


def main():
	
  d = {(0, 1): 'Treasure', (1, 0): 'Treasure', (0, 0): 'The Hero', (1, 1): 'The Dragon'}


  with open('result.yml', 'w') as yaml_file:
    yaml.dump(d, yaml_file, default_flow_style=False)



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
