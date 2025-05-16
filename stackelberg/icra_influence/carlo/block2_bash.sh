# Bash to run the randomized and balanced scenes automatically
# Order:
<<LatinSquare
H   R   I
R   I   H
I   H   R
LatinSquare
python3 highway.py
python3 roundabout.py
python3 intersection.py

python3 roundabout.py
python3 intersection.py
python3 highway.py

python3 intersection.py
python3 highway.py
python3 roundabout.py