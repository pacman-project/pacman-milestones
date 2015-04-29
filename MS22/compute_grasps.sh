#!/usr/bin/env bash

#Script:
#1. Lets you select a part to grasp from current scene and copies files into correct template folder (Loop with visualization of objects and parts)
#2. Runs grasp planner on current selection
#3. Executes computed grasp on simulated or real robot

PATH_TEMPLATES="/home/alexander/workspace/template_generator/pacman_pilot/generated_templates"
PATH_SCENE="/home/alexander/PHD/projects/PacMan/graspTemplateExperiment/scene"
PATH_PARTS="/home/alexander/PHD/projects/PacMan/graspTemplateExperiment/parts-recog"
SELECTED_OBJ=""

#load robot description onto parameter server if necessary:
#roslaunch uibk_robot_moveit_config robdes.launch

##OBJECT SELECTION AS ONLY ONE OBJ SKIP:
while true; do
    SELECTED_OBJ=""
    
    echo "Object List:"
    object=""
    objectfilelist=$(ls $PATH_PARTS)

    for object in $objectfilelist; do
      echo $object
      #currPath=$(basename $part)
    done

    echo "Please Select an Object:"
    #read SELECTED_OBJ
    echo "You entered: $SELECTED_OBJ"
    SELECTED_OBJ="scene_segmented" #hardcodede!
    #here could check for existing, not needed now!
    #pcl_viewer $PATH_PARTS/$SELECTED_OBJ/*/part.pcd    
    
    #read -p "Are you ok with this selection?" yn
    #case $yn in
    #    [Yy]* ) break;;
    #    [Nn]* ) break;; echo "Next";;
    #    * ) break;;echo "Next";;
    #esac
    break;
done
echo "You selected:"
echo $SELECTED_OBJ

##PART SELECTION:
OBJECT_PATH=$PATH_PARTS/$SELECTED_OBJ
SELECTED_PART=""
while true; do
    SELECTED_PART=""
    
    echo "Object List:"
    part=""
    partfilelist=$(ls $OBJECT_PATH)
    
    

    for part in $partfilelist; do
      echo "Part:"
      echo $part
      echo "Cluster_id:"
      cluster_id=$(head -c 1 $OBJECT_PATH/$part/id.txt) #take care no cluster_id more than 9 and no multiple ids !!!
      echo $cluster_id
      echo 
      #currPath=$(basename $part)
    done
    
    #yawan -l $OBJECT_PATH/*/part.pcd

    pcl_viewer -multiview 1 $OBJECT_PATH/*/part.pcd

    echo "Please Select a Partt:"
    read SELECTED_PART
    echo "You entered: $SELECTED_PART"
    echo "CLUSTER_ID:"
    cluster_id=$(head -c 1 $OBJECT_PATH/$SELECTED_PART/id.txt) #take care no cluster_id more than 9 and no multiple ids !!!
    echo $cluster_id

    #here could check for existing, not needed now!
    pcl_viewer $OBJECT_PATH/$SELECTED_PART/part.pcd   
    
    read -p "Are you ok with this selection?" yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) echo "Next";;
        * ) echo "Next";;
    esac
done
echo "You selected:"
echo $OBJECT_PATH/$SELECTED_PART

#####
####read cluster_id (for now only one) of current part and 
#copy the boundingbox-file and the scenefile to the correct grasp folder (later check out obst.pcd makes more sense?? lets see speed)
cluster_id=$(head -c 1 $OBJECT_PATH/$SELECTED_PART/id.txt) #take care no cluster_id more than 9 and no multiple ids !!!
echo $cluster_id

####only select grasp that match
#for testing i put 1
cluster_id=1
######

GRASP_FOLDER=$PATH_TEMPLATES/$cluster_id/scene
PROTOTYPE_DIR=$PATH_TEMPLATES/$cluster_id/prototypes

cp $OBJECT_PATH/$SELECTED_PART/part.bbox $PATH_TEMPLATES/$cluster_id/scene/part.bbox
#remove points below plane but not plane!
nuklei conv --box_roi "0.2 0.5 0.5 1 0 0 0 1 1 1" -w pcd $PATH_SCENE/scene.pcd $PATH_TEMPLATES/$cluster_id/scene/scene.pcd
#cp $PATH_SCENE/scene.pcd $PATH_TEMPLATES/$cluster_id/scene/scene.pcd
#for vizualization:
nuklei conv -w pcd --set_rgb "0 1 0" --scale 1000 $OBJECT_PATH/$SELECTED_PART/part.pcd $PATH_TEMPLATES/$cluster_id/scene/part.pcd

##now call planner
cd $PATH_TEMPLATES/$cluster_id
pbp-planner.py -d scene --config config.cfg -v 5

###executing grasp #for now no collission objects for pregrasp
#rosrun uibk_grasp_pipeline pacman_grasp_sm $GRASP_FOLDER/scores.txt $GRASP_FOLDER $PROTOTYPE_DIR

    




#pointcloudToBbox $part part.bbox
