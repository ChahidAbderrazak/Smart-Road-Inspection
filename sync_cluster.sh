cluster=graham #cedar #
START=$(date +%s)

##########################################################################
echo && echo " #################################################" 
echo " ##           IMAGE CLASSIFICATION              ##" 
echo " #################################################" && echo  

if [[ $cluster == "graham" ]]
then
###################### GRAHAM #############################
# Copy codes
echo  " ---> Exporting codes/files $cluster..." 
rsync -azP ../ abd100@$cluster.computecanada.ca:/home/abd100/Desktop/running/classification-single-input 
else
###################### CEDAR #############################
# Copy codes
echo  " ---> Exporting codes/files to $cluster..." 
rsync -axvHP --no-g --no-p  ../ abd100@$cluster.computecanada.ca:/home/abd100/projects/def-hgabbar/abd100/running/classification-single-input 
fi

# Import files
echo  " ---> Importing outputs to $cluster..." 
rsync -azP abd100@$cluster.computecanada.ca:/home/abd100/projects/def-hgabbar/out-abderrazak/  \
           /media/abdo2020/DATA1/running/tmp-cluster/

# # Copy compressed dataset :  tar -cf ../data.tar  *
# echo  " ---> Exporting dataset to $cluster..." 
# rsync  -axvHP --no-g --no-p   /media/abdo2020/DATA1/Datasets/images-dataset/labeled-data/classification-dataset/Carla_simulated_data/Carla-safety-index_2/Carla-safety-index_2.tar  \
#  			abd100@$cluster.computecanada.ca:/home/abd100/projects/def-hgabbar/dataset/classification/