function duckkeydrive(duck)
    h=figure;
    set(h,'KeyPressFcn',{@duckdrive,duck});    
end

function duckdrive(hobj,callbackdata,duck)
    v=1;w=2;T=.2;
    %display(callbackdata.Key);
    switch callbackdata.Key
        case{'uparrow'}
            duck.sendCmd(v,-2.85*w);
            pause(T)
            duck.sendCmd(0,0);            
        case{'downarrow'}
            duck.sendCmd(-v,2.85*w);
            pause(T)
            duck.sendCmd(0,0); 
        case{'leftarrow'};
            duck.sendCmd(0,1.5*w);
            pause(T)
            duck.sendCmd(0,0); 
        case{'rightarrow'};
            duck.sendCmd(0,-1.5*w);
            pause(T)
            duck.sendCmd(0,0); 
        case{'c'}
            n=921600;h=640;w=480;
            I1=duck.getImage();
            C=zeros(480,640,3,'uint8');
            C(:,:,1)=reshape(I1.data(1:3:n),h,w)';
            C(:,:,2)=reshape(I1.data(2:3:n),h,w)';
            C(:,:,3)=reshape(I1.data(3:3:n),h,w)';
            figure(100);image(C);colorbar;
        otherwise
            disp(callbackdata.Key)
    end
    end
