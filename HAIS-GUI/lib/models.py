import torchvision

# define the model instanciation
def get_model_instance(model_name):
    print('model_name=', model_name)
    try : 
        if model_name == 'RestNet50':
            # # Create an instance of the model class and allocate it to the device
            # model= Net(num_classes=len(classes))
            # load a model; pre-trained on COCO
            print('\n - Instantiate a <fasterrcnn_resnet50_fpn> model structure!')
            clf_model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
        return clf_model 

    except:
        msg = "The instantiation  of model architecture named <" +  model_name + ">  is not found"
        print(msg)
        raise Exception(msg)

######################## MODELS #########################
