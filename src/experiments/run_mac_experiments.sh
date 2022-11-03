
#-----------------------------------------------------------#
#       ___           ___           ___
#      /\__\         /\  \         /\  \
#     /::|  |       /::\  \       /::\  \
#    /:|:|  |      /:/\:\  \     /:/\:\  \
#   /:/|:|__|__   /::\~\:\  \   /:/  \:\  \
#  /:/ |::::\__\ /:/\:\ \:\__\ /:/__/ \:\__\
#  \/__/~~/:/  / \/__\:\/:/  / \:\  \  \/__/
#        /:/  /       \::/  /   \:\  \
#       /:/  /        /:/  /     \:\  \
#      /:/  /        /:/  /       \:\__\
#      \/__/         \/__/         \/__/
#
#-----------------------------------------------------------#

#test baselines
for nd in "2" "5" "8" "10" "15";  # number of drones
do 
    for alg in "RND" "RR" "AI";
    do 
        echo "run: ${alg} - ndrones ${nd} "
        python3 -m src.experiments.experiment_mac -nd ${nd} -i_s 0 -e_s 10 -alg ${alg} &
        #python3 -m src.experiments.experiment_mac -nd ${nd} -i_s 10 -e_s 20 -alg ${alg} &
        #python3 -m src.experiments.experiment_mac -nd ${nd} -i_s 20 -e_s 30 -alg ${alg} &
        #wait
    done;
    #wait
done; 
wait

python3 -m src.experiments.json_and_plot -nd 2 -nd 5 -nd 8 -nd 10 -nd 15  -i_s 0 -e_s 10 -exp_suffix RND -exp_suffix RR -exp_suffix AI
