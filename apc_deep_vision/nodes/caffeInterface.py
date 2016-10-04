# -*- coding: utf-8 -*-
"""
Created on Wed Jul  9 11:17:34 2014

@author: niko
"""

import numpy as np
#import caffe
import imp
import copy
import time

class CaffeNet:
  """This support the newer Caffe Python interface. The other class below (CaffeInterface) is deprecated
  and should not be used for new developments anymore."""
  
  def __init__(self, caffe_root=None, caffe_model_file=None, caffe_pretrained_file=None, caffe_mean_file=None, caffe_mean_vec=None):
    if caffe_root == None:
      caffe_root = '/home/niko/src/caffe/'

    if caffe_model_file == None:
      caffe_model_file = caffe_root + 'examples/imagenet/imagenet_deploy.prototxt'

    if caffe_pretrained_file == None:
      caffe_pretrained_file = caffe_root + 'examples/imagenet/caffe_reference_imagenet_model'

    if caffe_mean_file == None:
      caffe_mean_file = caffe_root + 'python/caffe/imagenet/ilsvrc_2012_mean.npy'
      

    try:
      print 'Loading Caffe module from', caffe_root,'...'
      filename, path, desc =  imp.find_module('caffe', [caffe_root+'/python/'])
      self.caffe = imp.load_module('caffe', filename, path, desc)
            
      self.caffe.set_mode_gpu()
      
      print 'Loading network from definition', caffe_model_file,'and network parameters', caffe_pretrained_file
      self.net = self.caffe.Net(caffe_model_file, caffe_pretrained_file, self.caffe.TEST)
            
      self.blob_names = self.net.blobs.keys()     
      self.input_name = self.blob_names[0]       
      self.output_name = self.blob_names[-1]       
            
      # set up transformer object                
      self.init_transformer(caffe_mean_vec, caffe_mean_file)
      
    except ImportError:
      print "\nError: Module caffe not found at path " + caffe_root + ". I try to continue, but you cannot use Caffe's functionality."
      self.net=None
    except Exception, e:
      print "Error:", e
      self.net=None
  
  # ========================================================================  
  def init_transformer(self, caffe_mean_vec=None, caffe_mean_file=None, blob_name=None):
    # input preprocessing: 'data' is the name of the input blob == net.inputs[0]
    
    if blob_name==None:
      blob_name = self.input_name
      
    self.transformer = self.caffe.io.Transformer({blob_name: self.net.blobs[blob_name].data.shape})
    self.transformer.set_transpose(blob_name, (2,0,1))
    if caffe_mean_vec==None:
      self.transformer.set_mean(blob_name, np.load(caffe_mean_file).mean(1).mean(1)) # mean pixel
    else:
      self.transformer.set_mean(blob_name, np.array(caffe_mean_vec)) # mean pixel
    # self.transformer.set_raw_scale(blob_name, 255)  # the reference model operates on images in [0,255] range instead of [0,1]
    self.transformer.set_channel_swap(blob_name, (2,1,0))  # the reference model has channels in BGR order instead of RGB


# ========================================================================  
# ========================================================================
# ========================================================================
    

class CaffeInterface:
    """ Old Python interface, started in July 2014, use the newer interface class above."""
    def __init__(self, caffe_root=None, caffe_model_file=None, caffe_pretrained_file=None, caffe_mean_file=None):

        if caffe_root == None:
          caffe_root = '/home/niko/src/caffe/'

        if caffe_model_file == None:
          caffe_model_file = caffe_root + 'examples/imagenet/imagenet_deploy.prototxt'

        if caffe_pretrained_file == None:
          caffe_pretrained_file = caffe_root + 'examples/imagenet/caffe_reference_imagenet_model'

        if caffe_mean_file == None:
          caffe_mean_file = caffe_root + 'python/caffe/imagenet/ilsvrc_2012_mean.npy'

        try:
            filename, path, desc =  imp.find_module('caffe', [caffe_root+'/python/'])
            caffe = imp.load_module('caffe', filename, path, desc)


            self.net = caffe.Classifier(caffe_model_file, caffe_pretrained_file)
            self.net.set_phase_test()
            self.net.set_mode_gpu()

            # input preprocessing: 'data' is the name of the input blob == net.inputs[0]
            try:
              self.net.set_mean('data', caffe_mean_file)  # ImageNet mean
            except:
              self.net.set_mean('data', np.load(caffe_mean_file)/255.0)  # ImageNet mean

            self.net.set_channel_swap('data', (2,1,0))  # the reference model has channels in BGR order instead of RGB
            self.net.set_input_scale('data', 255)  # the reference model operates on images in [0,255] range instead of [0,1]
        except ImportError:
            print "\nError: Module caffe not found at path " + caffe_root + ". I try to continue, but you cannot use Caffe's functionality."
            self.net=None
        except Exception, e:
            print "Error:", e
            self.net=None



    # ===============================================================
    def getFeatures(self, img, layer=None, oversample=True, image_dims=None, call_forward=False):

        if self.net==None:
            print 'Error: Trying to call CaffeInterface.getFeatures, but Caffe was not initialized correctly.'
            return

        # check inputs, img should be double and in range 0..1
        if type(img) == str:
            img = caffe.io.load_image(img)
        else:
            if img.dtype != 'float32':
                img = np.double(img)
            if img.max()>1.0:
                img = img / 255.0

        # make sure we have 3 input channels
        if img.ndim == 2:
          img = img[:, :, np.newaxis]
          img = np.tile(img, (1, 1, 3))
        elif img.shape[2] == 4:
          img = img[:, :, :3]

        # set the image size and crop sizes accordingly
        # when not doing oversampling, we want the whole image to be used for the
        # feature calculation, therefore we set net.image_dims to (227,227)
        # when oversampling, the call to predict first resizes the image to image_dims and then
        # extracts 10 crops (center + 4 corners and their mirriored versions) of size crop_dimthe

        if not oversample:
          if not image_dims:
            self.net.image_dims = np.asarray([227,227])
          else:
            self.net.image_dims = np.asarray(image_dims)
           # self.net.image_dims = np.asarray([256,256])   # !!!!!
        else:
            self.net.image_dims = np.asarray(img.shape[0:2])

        # now run it through the network
        #t1=time.time()

        if call_forward:
          fwd = self.net.forward(data=np.asarray([self.net.preprocess('data', img)]))
          if layer and layer in fwd.keys():
            return fwd[layer]
          else:
            return fwd
        else:
          self.net.predict([img], oversample)
          #print time.time()-t1

          if layer and layer in self.net.blobs.keys():
            return self.net.blobs[layer].data
          else:
            return self.net.blobs



#plt.rcParams['figure.figsize'] = (10, 10)
#plt.rcParams['image.interpolation'] = 'nearest'
#plt.rcParams['image.cmap'] = 'gray'



# ==========================================================================
# ==========================================================================
