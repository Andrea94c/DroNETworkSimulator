#-----------------------------------------------------------#
#           _  _ ___  ___  ___  _  _ ___ ___                #
#          | \| |   \| _ \/ _ \| \| | __/ __|               #
#          | .` | |) |   / (_) | .` | _|\__ \               #
#          |_|\_|___/|_|_\\___/|_|\_|___|___/               #
#                                                           #
#-----------------------------------------------------------#

#test baselines
for nd in "2" "5" "10" "15" "20" "30" "40";
do
    for alg in "GEO" "RND" "AI" "MGEO";
    # if you experienced too much time to run experiments, remove "GEO" and "RND"
    do 
        echo "run: ${alg} - ndrones ${nd} "
        python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 1 -e_s 3 -alg ${alg} &
        #python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 10 -e_s 20 -alg ${alg} &
        #python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 20 -e_s 30 -alg ${alg} &
    done;
done; 
wait

python3 -m src.experiments.json_and_plot -nd 2 -nd 5 -nd 10 -nd 15 -nd 20 -nd 30 -nd 40 -i_s 1 -e_s 3 -exp_suffix MGEO -exp_suffix GEO -exp_suffix RND -exp_suffix AI

