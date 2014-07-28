#!/bin/bash
DATA_PATH="/home/lestefan/datasets/mikolajczyk/" # set this...
DATASET="bikes"
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISK24000 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISKOLD45 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISKOLD45 BRISKOLD
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img1.pgm -oc 3 -thres 35000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-1.kpts
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img4.pgm -oc 3 -thres 35000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-4.kpts
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 SIFT80 SIFT

DATASET="boat"
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISK1300000 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISKOLD90 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISKOLD90 BRISKOLD
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img1.pgm -oc 3 -thres 800
mv out.surf $DATA_PATH$DATASET/SURF-SURF-1.kpts
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img4.pgm -oc 3 -thres 800
mv out.surf $DATA_PATH$DATASET/SURF-SURF-4.kpts
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 SIFT2.2 SIFT

DATASET="graf"
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 3 BRISK360000 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 3 BRISKOLD76 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 3 BRISKOLD76 BRISKOLD
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img1.pgm -oc 3 -thres 9000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-1.kpts
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img3.pgm -oc 3 -thres 9000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-3.kpts
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 3 SIFT3.8 SIFT

DATASET="leuven"
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISK200000 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISKOLD64 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISKOLD64 BRISKOLD
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img1.pgm -oc 3 -thres 22000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-1.kpts
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img4.pgm -oc 3 -thres 22000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-4.kpts
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 SIFT1.8 SIFT

DATASET="trees"
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISK380000 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISKOLD83 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISKOLD83 BRISKOLD
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img1.pgm -oc 3 -thres 50000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-1.kpts
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img4.pgm -oc 3 -thres 50000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-4.kpts
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 SIFT1.6 SIFT

DATASET="ubc"
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISK1550000 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISKOLD108 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISKOLD108 BRISKOLD
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img1.pgm -oc 3 -thres 50000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-1.kpts
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img4.pgm -oc 3 -thres 50000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-4.kpts
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 SIFT1.9 SIFT

DATASET="wall"
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISK290000 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISKOLD64 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 BRISKOLD64 BRISKOLD
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img1.pgm -oc 3 -thres 37000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-1.kpts
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img4.pgm -oc 3 -thres 37000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-4.kpts
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 4 SIFT1.85 SIFT

DATASET="rotwall"
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 BRISK290000 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 BRISKOLD71 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 BRISKOLD71 BRISKOLD
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img1.pgm -oc 3 -thres 55000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-1.kpts
./SURF-V1.0.9/surf.ln -i $DATA_PATH$DATASET/img2.pgm -oc 3 -thres 55000
mv out.surf $DATA_PATH$DATASET/SURF-SURF-2.kpts
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 SIFT1.5 SIFT

# this is for a second evaluation
DATASET="wall"
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 S-BRISKOLD63 SU-BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 S-BRISKOLD63 S-BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 BRISKOLD67 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 S-BRISKOLD63 BRIEF
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 S-BRISKOLD63 ORB
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 S-BRISKOLD63 SU-FREAK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 S-BRISKOLD63 S-FREAK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 BRISKOLD67 FREAK

DATASET="boat"
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 S-BRISKOLD94 SU-BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 S-BRISKOLD94 S-BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 BRISKOLD115 BRISK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 S-BRISKOLD94 BRIEF
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 S-BRISKOLD94 ORB
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 S-BRISKOLD94 SU-FREAK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 S-BRISKOLD94 S-FREAK
../brisk/bin/mikolajczyk $DATA_PATH$DATASET 2 BRISKOLD115 FREAK
