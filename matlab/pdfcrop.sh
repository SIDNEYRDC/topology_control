#!/bin/bash

for i in $(ls *.pdf); do
    mv $i "$i".tmp
    pdfcrop "$i".tmp $i
    rm "$i".tmp
done

