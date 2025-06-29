for ((seed=1011; seed<1021; seed++)); do
    echo "Running with seed $seed"
    ./MOEAD "$seed"
done