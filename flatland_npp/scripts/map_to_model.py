import cv2
import argparse
import imutils
import csv
import numpy as np
import os

def read_model_list(fname, model_list):
    # read in csv file with rows: model_name, r, g, b
    with open(fname, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            model_list.append(row)

    return model_list

def get_contours(img, model_list):
    # append contours to model_list
    for i in range(len(model_list)):
        model = model_list[i]
        r = int(model[1])
        g = int(model[2])
        b = int(model[3])
        # thresh = np.zeros(img.shape)
        # thresh[img[:,:,0] == r and img[:
        thresh = cv2.inRange(img, (b, g, r), (b, g, r))
        # cv2.imshow('img', thresh)
        # cv2.waitKey(0)

        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)[0]
        # convert to a numpy array
        cnts = np.array(cnts)[:,0,:]
        cnts = cnts.astype('float64')
        model_list[i].append(cnts)
    return model_list

def populate_modelfile(fname, model):
    with open(fname, 'r') as f:
        lines = f.readlines()

    for i in range(0, len(lines)):
        if 'name:' in lines[i]:
            lines[i] = lines[i][:-1] + ' ' + model[0] + '\n'
        if 'points:' in lines[i]:
            lines[i] = lines[i] + ' ' + np.array2string(model[4], separator=', ') + '\n'

    return lines

def generate_models(imfile, csvfile, outdir, resolution, origin):
    model_list = []
    # read in csv to generate list of models corresponding to rgb values
    model_list = read_model_list(csvfile, model_list)

    # read in image file
    img = cv2.imread(imfile)
    h, w, c = img.shape

    # get contours in image file, thresholding on rgb values
    model_list = get_contours(img, model_list)

    # convert contours to polygon points for flatland model
    for i in range(0, len(model_list)):
        coords = model_list[i][4]
        coords[:,0] = (w - coords[:,0])*resolution + origin[0]
        coords[:,1] = (h - coords[:,1])*resolution + origin[1]

    # generate flatland model files
    # read in a template file template.model.yaml
    template_file = 'template.model.yaml'
    for model in model_list:
        lines = populate_modelfile(template_file, model)
        model_file = os.path.join(outdir, model[0] + '.model.yaml')
        with open(model_file, 'w') as f:
            f.writelines(lines)

parser = argparse.ArgumentParser()
parser.add_argument('imfile')
parser.add_argument('csvfile')
parser.add_argument('outdir')
parser.add_argument('resolution', type=float, help='Resolution in meters/pixel. Should match map resolution')
parser.add_argument('--origin', nargs=2, type=float, help='Origin of map in map.yaml')

args = parser.parse_args()
generate_models(args.imfile, args.csvfile, args.outdir, args.resolution, args.origin)
