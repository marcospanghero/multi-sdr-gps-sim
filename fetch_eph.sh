#!/bin/sh
day=$(date +%j)
year=$(date +%Y)
yr=$(date +%y)


echo "brdc""$day""0.$yr""n.gz" "https://cddis.nasa.gov/archive/gnss/data/daily/$year""/brdc/brdc""$day""0.$yr""n.gz"

curl -c /tmp/cookie -n -L -o "brdc""$day""0.$yr""n.gz" "https://cddis.nasa.gov/archive/gnss/data/daily/$year""/brdc/brdc""$day""0.$yr""n.gz"


uncompress "brdc""$day""0.$yr""n.gz"
echo "brdc""$day""0.$yr""n.gz"