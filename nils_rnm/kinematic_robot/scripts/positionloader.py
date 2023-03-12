import numpy as np
import os 




def creator():
    liste_cc = np.load(os.path.join(os.path.dirname(__file__),'Position/collected_joint_cc.npy'))
    liste_mr = np.load(os.path.join(os.path.dirname(__file__),'Position/collected_joint_list_mr.npy'))
    list_cc = liste_cc[:2]
    list_ap = np.append(list_cc, liste_mr)

    np.save(os.path.join(os.path.dirname(__file__),'Position/dummy_list.npy'),list_ap)

creator()