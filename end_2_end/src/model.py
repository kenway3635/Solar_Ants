import torch 
import torchvision.transforms as T
import torch.nn as nn

def dwpw_conv(in_channels, out_channels, kernel_size, stride=1, padding=0):
    return nn.Sequential(
        nn.Conv2d(in_channels, in_channels, kernel_size, stride=stride, padding=padding, groups=in_channels), #depthwise convolution
        nn.Conv2d(in_channels, out_channels, 1), # pointwise convolution
    )
class Model_0718(nn.Module):
    def __init__(self, input_dim):
        super(Model_0718, self).__init__()
        #apply image argumention
        
        if self.training == True:
            self.transform = nn.Sequential(
                T.RandomCrop((96,256)),
                T.RandomApply(torch.nn.ModuleList([
                    T.ColorJitter(brightness=0.2,contrast= 0.1,saturation=0.1,hue=0.1),
                    #T.RandomRotation((-10,10)),
                    #T.ColorJitter(hue=0.3),
                    T.RandomErasing(0.3),
                    T.RandomInvert(0.3),
                    T.RandomGrayscale(0.1),
                    T.GaussianBlur((5,5)),
                ]), p=0.5),
                T.ConvertImageDtype(torch.float),
                T.Normalize(mean = (0.5, 0.5, 0.5), std = (0.5, 0.5, 0.5))
            )
        elif self.training == False:
            self.transform = nn.Sequential(
                T.RandomCrop((96,256)),
                T.ConvertImageDtype(torch.float),
                T.Normalize(mean = (0.5, 0.5, 0.5), std = (0.5, 0.5, 0.5))
            )
        
        
        #image CNN layers
        self.cnn = nn.Sequential(
            
            dwpw_conv(3, 64, 3, 1, 1), # [32, 96, 256]
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.MaxPool2d(2, 2, 0),      # [32, 48, 128]

            dwpw_conv(64, 128, 3, 1, 1), # [64, 48, 128]
            nn.ReLU(),
            nn.BatchNorm2d(128),
            nn.MaxPool2d(2, 2, 0),      # [64, 24, 64]
            
            dwpw_conv(128, 128, 3, 1, 1), # [128, 24, 64]
            nn.ReLU(),
            nn.BatchNorm2d(128),
            nn.MaxPool2d(2, 2, 0),      # [128, 12, 32]
            
            dwpw_conv(128, 256, 3, 1, 1), # [256, 12, 32]
            nn.ReLU(),
            nn.BatchNorm2d(256),
            nn.MaxPool2d(2, 2, 0),# [256, 6, 16]
            
            dwpw_conv(256, 256, 3, 1, 1), # [256, 12, 32]
            nn.ReLU(),
            nn.BatchNorm2d(256),
            nn.AvgPool2d(2,2,0)
        )
        
        #image softmax layer
        self.fc = nn.Sequential(
        #nn.Linear(256*6*16,1024),
        nn.Linear(6144,1024),
        nn.ReLU(),
        nn.Linear(1024, 256),
        nn.ReLU(),
        nn.Linear(256, 64),
        nn.ReLU(),
        nn.Linear(64, 10),
        )
        '''
        #IMU fully connected layer
        self.IMUlayers = nn.Sequential(
            nn.Linear(10, 10),
            nn.Dropout(0.1),
            nn.ReLU(),
            nn.Linear(10, 10),
            nn.Dropout(0.1),
            nn.ReLU(),
            nn.Linear(10, 10),
        )
        '''
        #the last fully connected layer
        self.lastlayers = nn.Sequential(
            nn.Linear(20, 20),
            nn.ReLU(),
            nn.Linear(20, 20),
            nn.ReLU(),
            nn.Linear(20, 10),
            nn.ReLU(),
            nn.Linear(10, 2),
            
        )
        
        
        #recurrent layers
       # self.rnn=nn.RNN(input_size=20,hidden_size=20,num_layers=1,batch_first=True)
        
       # self.lstm = nn.LSTM(input_size=20, hidden_size=20, num_layers=1,batch_first=True)
        '''
        self.cnn_layer1 = nn.Sequential(
            dwpw_conv(3, 32, 3, 1, 1),
            nn.BatchNorm2d(32),
            nn.MaxPool2d(2, 2, 0),
        )
        self.cnn_layer2 = nn.Sequential(
            dwpw_conv(32, 32, 3, 1, 1),
            nn.BatchNorm2d(32),
        )

        self.cnn_layer3 = nn.Sequential(
            dwpw_conv(32, 64, 3, 2, 1),
            nn.BatchNorm2d(64),
           # nn.MaxPool2d(2, 2, 0),
        )

        self.cnn_layer4 = nn.Sequential(
            dwpw_conv(64, 64, 3, 1, 1),
            nn.BatchNorm2d(64),
        )
        self.cnn_layer5 = nn.Sequential(
            dwpw_conv(64, 128, 3, 2, 1),
            nn.BatchNorm2d(128),
            #nn.MaxPool2d(2, 2, 0),
        )
        self.cnn_layer6 = nn.Sequential(
            dwpw_conv(128, 128, 3, 1, 1),
            nn.BatchNorm2d(128),
        )
        self.cnn_layer7 = nn.Sequential(
            dwpw_conv(128, 128, 3, 1, 1),
            nn.BatchNorm2d(128),
            #nn.MaxPool2d(2, 2, 0),
        )
        self.cnn_layer8 = nn.Sequential(
            dwpw_conv(128, 128, 3, 1, 1),
            nn.BatchNorm2d(128),
        )
        self.cnn_layer9 = nn.Sequential(
            dwpw_conv(128, 128, 3, 1, 1),
            nn.BatchNorm2d(128),
            nn.AvgPool2d(2, 2, 0),
        )
        
        self.relu = nn.ReLU()
        '''
        
    def forward(self, x):
        #reshape 1-D vector to RGB image tensor
        x1 = torch.reshape((x[:,:-10]),(x.size(0),180,320,3))# to 128x128 image
        x1=x1.permute(0,3,1,2)
        #apply image transform
        x1=self.transform(x1)

        '''
        img = x1[0].cpu()
        npimg = img.numpy().astype('uint8')
        plt.imshow(np.transpose(npimg, (1,2,0)), interpolation='nearest')
        '''
        #Residual CNN
        
        #image layers
        x1=self.cnn(x1)

        #x1 = x1.view(x1.size()[0], -1)
        x1=torch.flatten(x1, start_dim=1)
        x1=self.fc(x1)
        
        

        #odometry layers
        x2 = torch.reshape((x[:,49153]),(x.size(0),1))
        
        x2 = x[:,-10:]
        #x2 = self.IMUlayers(x2)
        x=torch.cat((x1,x2),1)
        
        #x=x1
        
        #h0=torch.zeros(1,x.size(0),x.size(1)).to(device)
        #c0=torch.zeros(1,x.size(0),x.size(1)).to(device)
        #x = x.view(x.size(0), x.size(1), -1)
        #x = x.permute(0, 2, 1)
        #x,_=self.lstm(x,(h0,c0))
        #x,_=self.rnn(x,h0)
        #x=x.reshape(x.shape[0],-1)  #for rnn 
        #x=x[:,-1,:] # for lstms
        
        
        #last fully connected layer
        x = self.lastlayers(x)
        


        return x


class Visual_IMU_Model(nn.Module):
    def __init__(self, input_dim):
        super(Visual_IMU_Model, self).__init__()
        #apply image argumention
        self.transform = nn.Sequential(
            T.RandomCrop((100,300)),
            T.ConvertImageDtype(torch.float),
            T.Normalize(mean = (0.5, 0.5, 0.5), std = (0.5, 0.5, 0.5))
        )
        
        
        #image CNN layers
        self.cnn = nn.Sequential(
            
            dwpw_conv(3, 16, 5, 1, 1), # [64, 96, 96]
            nn.ReLU(),
            #nn.BatchNorm2d(24),
            #nn.MaxPool2d(2, 2, 0),      # [64, 48, 48]

            dwpw_conv(16, 32, 5, 1, 1), # [128, 48, 48]
            nn.ReLU(),
            #nn.BatchNorm2d(36),
            #nn.MaxPool2d(2, 2, 0),      # [128, 24, 24]
            
            dwpw_conv(32, 32, 3, 1, 1), # [128, 48, 48]
            nn.ReLU(),
            #nn.BatchNorm2d(48),
            nn.MaxPool2d(2, 2, 0),      # [128, 12, 12]
            
            dwpw_conv(32, 16, 3, 1, 1), # [128, 12, 12]
            nn.ReLU(),
            #nn.BatchNorm2d(96),
            nn.MaxPool2d(2, 2, 0),      # [128, 6, 6]
        )
        
        #image softmax layer
        self.fc = nn.Sequential(
        nn.Linear(28416,1000),
        nn.Dropout(0.1),
        nn.ReLU(),
        nn.Linear(1000, 100),
        nn.Dropout(0.1),
        nn.ReLU(),
        nn.Linear(100, 50),
        nn.Dropout(0.1),
        nn.ReLU(),
        nn.Linear(50, 10),
        )
        #IMU fully connected layer
        self.IMUlayers = nn.Sequential(
            nn.Linear(10, 256),
            nn.Dropout(0.2),
            nn.ReLU(),
            nn.Linear(256, 64),
            nn.Dropout(0.2),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.Dropout(0.2),
            nn.ReLU(),
            nn.Linear(32, 10),
        )
        #recurrent layers
        self.rnn=nn.RNN(input_size=10,hidden_size=10,num_layers=1,batch_first=True)
        
        self.lstm = nn.LSTM(input_size=10, hidden_size=10, num_layers=1,batch_first=True)
        
        #the last fully connected layer
        self.lastlayers = nn.Sequential(
            nn.Linear(20, 256),
            nn.Dropout(0.1),
            nn.ReLU(),
            nn.Linear(256, 64),
            nn.Dropout(0.1),
            nn.ReLU(),
            nn.Linear(64, 16),
            nn.Dropout(0.1),
            nn.ReLU(),
            nn.Linear(16, 2),
        )
    def forward(self, x):
        #reshape 1-D vector to RGB image tensor
        x1 = torch.reshape((x[:,:-10]),(x.size(0),180,320,3))# to 128x128 image
        x1=x1.permute(0,3,1,2)
        #apply image transform
        x1=self.transform(x1)
              
        
        #image layers
        x1=self.cnn(x1)
        #x1 = x1.view(x1.size()[0], -1)
        x1=torch.flatten(x1, start_dim=1)
        x1=self.fc(x1)
        
        

        #odometry layers
        #x2 = torch.reshape((x[:,49153]),(x.size(0),1))
        
        x2 = x[:,-10:]
        x2 = self.IMUlayers(x2)
        x=torch.cat((x1,x2),1)
        
        #x=x1
        '''
        h0=torch.zeros(1,x.size(0),x.size(1)).to(device)
        c0=torch.zeros(1,x.size(0),x.size(1)).to(device)
        x = x.view(x.size(0), x.size(1), -1)
        x = x.permute(0, 2, 1)
        x,_=self.lstm(x,(h0,c0))
        #x,_=self.rnn(x,h0)
        #x=x.reshape(x.shape[0],-1)  #for rnn 
        x=x[:,-1,:] # for lstms
        '''
        
        #last fully connected layer
        x = self.lastlayers(x)
        


        return x

class CNN_ODOM_Model(nn.Module):
    def __init__(self, input_dim):
        super(CNN_ODOM_Model, self).__init__()
        #apply image argumention
        
        if self.training == True:
            self.transform = nn.Sequential(
                T.RandomCrop(96),
                T.RandomApply(torch.nn.ModuleList([
                    T.ColorJitter(),
                    #T.RandomRotation((-15,15)),
                    T.ColorJitter(hue=0.3),
                    T.RandomErasing(0.3),
                    T.RandomInvert(0.3),
                    T.RandomGrayscale(0.3),
                    T.GaussianBlur((3,3)),
                ]), p=0.5),
                T.ConvertImageDtype(torch.float),
                T.Normalize(mean = (0.5, 0.5, 0.5), std = (0.5, 0.5, 0.5))
            )
        elif self.training == False:
            self.transform = nn.Sequential(
                T.RandomCrop(96),
                T.ConvertImageDtype(torch.float),
                T.Normalize(mean = (0.5, 0.5, 0.5), std = (0.5, 0.5, 0.5))
            )
        
        
        #image CNN layers
        self.cnn = nn.Sequential(
            
            nn.Conv2d(3, 24, 5, 1, 1), # [64, 96, 96]
            #nn.BatchNorm2d(24),
            nn.ReLU(),
            nn.MaxPool2d(2, 2, 0),      # [64, 48, 48]

            nn.Conv2d(24, 36, 5, 1, 1), # [128, 48, 48]
            #nn.BatchNorm2d(36),
            nn.ReLU(),
            nn.MaxPool2d(2, 2, 0),      # [128, 24, 24]
            
            nn.Conv2d(36, 48, 3, 1, 1), # [128, 48, 48]
            #nn.BatchNorm2d(48),
            nn.ReLU(),
            nn.MaxPool2d(2, 2, 0),      # [128, 12, 12]
            
            nn.Conv2d(48, 96, 3, 1, 1), # [128, 12, 12]
            #nn.BatchNorm2d(96),
            nn.ReLU(),
            nn.MaxPool2d(2, 2, 0),      # [128, 6, 6]
        )
        
        #image softmax layer
        self.fc = nn.Sequential(
        nn.Linear(96*5*5,1000),
        nn.ReLU(),
        nn.Linear(1000, 100),
        nn.ReLU(),
        nn.Linear(100, 50),
        nn.ReLU(),
        nn.Linear(50, 10),
        )
        #odometry fully connected layer
        self.odomlayers = nn.Sequential(
            nn.Linear(3, 10),
            nn.ReLU(),
            nn.Linear(10, 10),
            nn.ReLU(),
            nn.Linear(10, 10),
            nn.ReLU(),
            nn.Linear(10, 10),
        )
        #recurrent layers
        self.rnn=nn.RNN(input_size=10,hidden_size=10,num_layers=1,batch_first=True)
        
        self.lstm = nn.LSTM(input_size=10, hidden_size=10, num_layers=1,batch_first=True)
        
        #the last fully connected layer
        self.lastlayers = nn.Sequential(
            nn.Linear(20, 2), #return 2
        )
    def forward(self, x):
        #reshape 1-D vector to RGB image tensor
        x1 = torch.reshape((x[:,:-3]),(x.size(0),128,128,3))# to 128x128 image
        x1=x1.permute(0,3,1,2)
        #apply image transform
        x1=self.transform(x1)

        '''    
        img = x1[0].cpu()
        npimg = img.numpy().astype('uint8')
        plt.imshow(np.transpose(npimg, (1,2,0)), interpolation='nearest')
        '''      
        
        #image layers
        x1=self.cnn(x1)
        #x1 = x1.view(x1.size()[0], -1)
        x1=torch.flatten(x1, start_dim=1)
        x1=self.fc(x1)
        
        

        #odometry layers
        #x2 = torch.reshape((x[:,49153]),(x.size(0),1))
        
        x2 = x[:,-3:]
        x2 = self.odomlayers(x2)
        x=torch.cat((x1,x2),1)
        
        #x=x1
        '''
        h0=torch.zeros(1,x.size(0),x.size(1)).to(device)
        c0=torch.zeros(1,x.size(0),x.size(1)).to(device)
        x = x.view(x.size(0), x.size(1), -1)
        x = x.permute(0, 2, 1)
        x,_=self.lstm(x,(h0,c0))
        #x,_=self.rnn(x,h0)
        #x=x.reshape(x.shape[0],-1)  #for rnn 
        x=x[:,-1,:] # for lstms
        '''
        
        #last fully connected layer
        x = self.lastlayers(x)
        


        return x