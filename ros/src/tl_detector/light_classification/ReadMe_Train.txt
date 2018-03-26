To re-train the model inside the udacity VM:

1) get image sources from https://www.dropbox.com/sh/w7qha7sqtzen48a/AABPBrTlwkKAL-JLb6OinW3Xa?dl=0 
and save in folder /ros/src/tl_detector/light_classification/Traffic_light


2) install proper jupyter for python 2:

pip install 'ipython<6.0' jupyter
pip install keras==1.2.2


3) run the notebook /ros/src/tl_detector/light_classification/Train.ipynb:

jupyter notebook  (open the browser -> find Train.ipynb and run the cell)


4) cp the model to the proper place:
(assuming you're in the repo main directory)

cp ./ros/src/tl_detector/light_classification/light_classifier_model.h5 ./ros/src/tl_detector/


