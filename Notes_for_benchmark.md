Todo.

1. Download davis 2016 and 2017 datasets. (Should provide short but high quality segmented images)

    Can download both from website or using the script in the git page.
    https://davischallenge.org/

    https://github.com/davisvideochallenge/davis-2017

Davis 2016 is only single object (single target) tracked.

Davis 2017 has single and multiple objects tracked.
 
Using the the script with adjusted folder name :
-  davis_evaluator.py (right now the folder name is hardcoded, changing to argument would be easier?)

currently running as:
- python davis_evaluator.py --input_path ./inputs/pick_and_place.mp4 --output_path ./outputs/videos/output_video.mp4 --save_all_mode --save_all_path  ./outputs/all_data  
Need to remove the unneeded arguments for simplicity...

load the images from the dataset (i.e. bear), select target in first image with sam, and store the masks.
    (get fps from here?)
Next run the 
- compute_j_and_f_scores.py to obtain J&F metrics 
    (get the J and F scores) from here

Proceed for the next folder in the davis dataset.

Once done we can compute the average FPS, J&F for davis 2016 and davis 2017.

The folders to used are 
for Davis 2016, from val.txt:

blackswan
bmx-trees*sharin
breakdance*boushi
camel
car-roundabout
car-shadow
cows
dance-twirl
dog
drift-chicane
drift-straight
goat
horsejump-high *
kite-surf *
libby
motocross-jump
paragliding-launch *
parkour
scooter-black
soapbox

for Davis 2017, from val.txt:
bike-packing
blackswan
bmx-trees *sharin
breakdance*boushi
camel
car-roundabout
car-shadow
cows
dance-twirl
dog
dogs-jump *ashi
drift-chicane
drift-straight
goat
gold-fish*order
horsejump-high
india
judo
kite-surf *iregguler order
lab-coat
libby
loading
mbike-trick
motocross-jump ** miss
paragliding-launch **head
parkour
pigs
scooter-black
shooting
soapbox

Note that that davis 2017 has multi-object data, so adjusting the codes
- 1. davis_evaluator.py
- 2. compute_j_and_f_scores.py
 is needed to obtain scores for the multiple targets: 
        1. adjust 2 to read red mask as object 1, green as object 2;
        2. provide the necessary path to load the annotated images provided for ist_ros
        3. combine the scores of J&F metrics to obtain final values.

Another suggestion would be to adjust the compute_j_and_f_scores.py, to open the txt file and store the evaluation values
or write a function to open the dataset and annotations from ist_ros, loop through the folders, evaluate and store the values.

There could be a better a way to do this, but this a short scripts I came up with in a short time.
    