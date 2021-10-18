#-----------------------------------------------------------#
#           _  _ ___  ___  ___  _  _ ___ ___                #
#          | \| |   \| _ \/ _ \| \| | __/ __|               #
#          | .` | |) |   / (_) | .` | _|\__ \               #
#          |_|\_|___/|_|_\\___/|_|\_|___|___/               #
#                                                           #
#-----------------------------------------------------------#

#test baselines
for nd in "5" "10" "15" "20" "25" "30" "35" "40";
do 
    for alg in "BROADCAST" "MOVE" "MOVE_W_COM";
    do 
        echo "run: ${alg} - ndrones ${nd} "
        python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 0 -e_s 10 -alg ${alg} &
        python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 10 -e_s 20 -alg ${alg} &
        python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 20 -e_s 30 -alg ${alg} &
    done;
done; 
wait

#test others algorithms
for nd in "5" "10" "15" "20" "25" "30" "35" "40";
do
    for alg in "QMR" "GFG" "BATMAN";
    do
        echo "run: ${alg} - ndrones ${nd} "
        python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 0 -e_s 10 -alg ${alg} &
        python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 10 -e_s 20 -alg ${alg} &
        python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 20 -e_s 30 -alg ${alg} &
    done;
done;
wait

python3 -m src.experiments.json_and_plot -nd 5 -nd 10 -nd 15 -nd 20 -nd 25 -nd 30 -nd 35 -nd 40 -i_s 1 -e_s 30 -exp_suffix MOVE -exp_suffix BROADCAST -exp_suffix MOVE_W_COM -exp_suffix QMR -exp_suffix BATMAN -exp_suffix GFG


