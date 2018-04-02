To re-train the model inside the udacity VM:

1) get image sources from: https://we.tl/lqH9SavwjK

and save the folders of the zip-file to: <repo-directory>/ros/src/tl_detector/light_classification/


2) copy all content from sub-directories 'Traffic_light_sim_only' and 'Traffic_light_real_only' into the empty directory 'Traffic_light_both'


3) install proper jupyter for python 2:

pip install 'ipython<6.0' jupyter
pip install keras==1.2.2


4) run the notebook /ros/src/tl_detector/light_classification/Train.ipynb:

jupyter notebook  (open the browser -> find Train.ipynb and run the cell)


5) copy the model to the proper place:

cp <repo-directory>/ros/src/tl_detector/light_classification/{path-name}light_classifier_model.h5 ./ros/src/tl_detector/light_classifier_model.h5


