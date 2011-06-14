#!/bin/bash

#rosservice call seds_server ["/home/stober/workspace/ros/seds/data/test.ml"] 0 "model.txt"
rosservice call seds_server ["/home/stober/workspace/ros/seds/turtle/bag0.bag"] 1 "model.txt"