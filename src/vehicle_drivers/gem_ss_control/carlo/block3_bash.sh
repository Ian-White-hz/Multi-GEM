# Bash to run the randomized and balanced scenes automatically
# Order:
<<LatinSquare
H   R   I
I   H   R
R   I   H
LatinSquare

python3 highway.py
python3 roundabout.py
python3 intersection.py

python3 intersection.py
python3 highway.py
python3 roundabout.py

python3 roundabout.py
python3 intersection.py
python3 highway.py