#/bin/bash

for i in {1..16}; do sed s/^/$i,/g data`printf "%02d" $i`.csv > with_ids/data`printf "%02d" $i`.csv; done
