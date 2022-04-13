import cv2
import argparse
import imutils
import csv
import numpy as np
import os
import yaml

def read_model_list(fname, model_list):
    # read in csv file with rows: model_name, r, g, b
    with open(fname, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            model_list.append(row)

    return model_list

def get_contours(img, model_list):
    h, w, c = img.shape
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
        cnts = imutils.grab_contours(cnts)
        # convert to a numpy array
        cnts = np.array(cnts)
        cnts = np.reshape(cnts, (cnts.shape[0], cnts.shape[1], cnts.shape[-1]))
        cnts = cnts.astype('float64')

        # add an extra pixel of padding
        for j in range(0, cnts.shape[0]):
            shape = cnts[j,:,:]
            xind = np.where(shape[:,0] != np.min(shape[:,0]))
            yind = np.where(shape[:,1] != np.min(shape[:,1]))
            shape[xind,0] = shape[xind,0] + 1
            shape[yind,1] = shape[yind,1] + 1

        # use first contour as object shape
        shape = cnts[0,:,:]
        model_list[i].append(shape)

        # add origin of each instance to the list
        orig = []
        for j in range(0, cnts.shape[0]):
            orig.append(np.mean(cnts[j,:,:], axis=0))

        model_list[i].append(orig)

        # cv2.imshow('img', cv2.circle(img, tuple(orig.astype(int)), radius=0, color=(0, 0, 255), thickness=1))
        # cv2.waitKey(0)
    return model_list

def get_line_points(model):
    # given the model shape, return the line along the rectangle's long axis
    line_points = np.array([[0.0, 0.0], [0.0, 0.0]])

    # shape = np.array((4,2)) for four (x,y) pairs
    shape = model[7] # get four points of the rectangle
    xdiff = np.max(shape[:,0]) - np.min(shape[:,0])
    ydiff = np.max(shape[:,1]) - np.min(shape[:,1])

    if (xdiff > ydiff):
        line_points[0,0] = np.min(shape[:,0])
        line_points[1,0] = np.max(shape[:,0])
    else:
        line_points[0,1] = np.min(shape[:,1])
        line_points[1,1] = np.max(shape[:,1])

    return line_points

def populate_modelfile(fname, model):
    with open(fname, 'r') as f:
        model_yaml = yaml.safe_load(f)

        model_yaml['bodies'][0]['name'] = model[0]
        model_yaml['bodies'][0]['footprints'][0]['points'] = model[7].tolist()
        model_yaml['plugins'][0]['enabled'] = model[4]
        model_yaml['plugins'][0]['body'] = model[0]
        model_yaml['plugins'][0]['geometry'] = model[5]
        model_yaml['plugins'][0]['value'] = model[6]
        line_points = get_line_points(model)
        model_yaml['plugins'][0]['line_points'] = line_points.tolist()

        return model_yaml

def populate_modellist(model_list):
    lines = []
    for model in model_list:
        modnum = 0
        for origin in model[8]:
            lines.append('  - name: {}{}\n'.format(model[0], modnum))
            lines.append('    pose: [{}, {}, 0.0]\n'.format(origin[0], origin[1]))
            lines.append('    model: {}.model.yaml\n'.format(model[0]))
            modnum += 1

    return lines

def generate_models(imfile, csvfile, outdir, worldfile, resolution, origin):
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
        model_list[i][7] = (model_list[i][7] - model_list[i][8][0])*resolution

        # convert points to world coordinates
        for j in range(0, len(model_list[i][8])):
            loc = model_list[i][8][j]
            loc[0] = loc[0]*resolution + origin[0]
            loc[1] = (h - loc[1])*resolution + origin[1]

    # generate flatland model files
    # read in a template file template.model.yaml
    template_file = os.path.join(os.path.dirname(__file__), 'template.model.yaml')
    for model in model_list:
        model_yaml = populate_modelfile(template_file, model)
        model_file = os.path.join(outdir, model[0] + '.model.yaml')
        with open(model_file, 'w') as f:
            yaml.dump(model_yaml, f)
            # f.writelines(lines)

    # generate model list
    # world_modellist = 'modellist.world.yaml'
    lines = populate_modellist(model_list)
    # modellist_file = os.path.join(outdir, world_modellist)
    with open(worldfile, 'a') as f:
        f.write('# added from map_to_model script\n')
        f.writelines(lines)

parser = argparse.ArgumentParser()
parser.add_argument('imfile')
parser.add_argument('csvfile')
parser.add_argument('outdir')
parser.add_argument('worldfile')
parser.add_argument('resolution', type=float, help='Resolution in meters/pixel. Should match map resolution')
parser.add_argument('--origin', nargs=2, type=float, help='Origin of map in map.yaml')

args = parser.parse_args()
generate_models(args.imfile, args.csvfile, args.outdir, args.worldfile, args.resolution, args.origin)
