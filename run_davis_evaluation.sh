#!/bin/bash

# DAVIS 2016 folders
davis_2016=(
    blackswan
    bmx-trees
    breakdance
    camel
    car-roundabout
    car-shadow
    cows
    dance-twirl
    dog
    drift-chicane
    drift-straight
    goat
    horsejump-high
    kite-surf
    libby
    motocross-jump
    paragliding-launch
    parkour
    scooter-black
    soapbox
)

# DAVIS 2017 folders
davis_2017=(
    bike-packing
    blackswan
    bmx-trees
    breakdance
    camel
    car-roundabout
    car-shadow
    cows
    dance-twirl
    dog
    dogs-jump
    drift-chicane
    drift-straight
    goat
    gold-fish
    horsejump-high
    india
    judo
    kite-surf
    lab-coat
    libby
    loading
    mbike-trick
    motocross-jump
    paragliding-launch
    parkour
    pigs
    scooter-black
    shooting
    soapbox
)

# Run evaluator for DAVIS 2016
echo "Evaluating DAVIS 2016..."
for folder in "${davis_2016[@]}"
do
    echo "Processing $folder..."
    python davis_evaluator.py --davis_year 2016 --davis_task "$folder"
done

# Run evaluator for DAVIS 2017
echo "Evaluating DAVIS 2017..."
for folder in "${davis_2017[@]}"
do
    echo "Processing $folder..."
    python davis_evaluator.py --davis_year 2017 --davis_task "$folder"
done

echo "Evaluation complete."