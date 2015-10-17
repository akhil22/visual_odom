function BundleSimpleFull01()
% This function is developed by M. Davoodianidaliki as part of MSc thesis.
% Everyone can use it as long as they mention author name.
% Contact me by mdavoodian@ut.ac.ir, mahmooddavoodian@gmail.com
% Use Help.txt file which is part of .zip file for more information

load BundleFull_911224_S06_Dual_Chk1.mat

close all
loop=true;
State=2; 
CamUnNum = 10;
StationUnNum = 6;
CamNum = length(CamData);
ChkUnNum=3;

RGBImgNum=length(StationData);

handle1=figure();tablenames{RGBImgNum*6+4}='';
for rtr=1:RGBImgNum
    tablenames{rtr*6-5+CamUnNum*CamNum}=['S',num2str(rtr)];
end
table1 = uitable(handle1,'rowname',tablenames,'position',[0,0,800,600]);
looprtr = 0;
rowrtr = 0;
Xmatcell=[];

Cntrl = PointData(1).Cntrl;
ChkInds=find(~Cntrl);ChkNum=length(ChkInds);ChkIndX=zeros(1,length(ChkInds));ChkIndZ=ChkIndX;
for Chktrtr=1:ChkNum
ChkIndX(ChkInds(Chktrtr)) = StationUnNum*RGBImgNum+CamNum*CamUnNum+(Chktrtr*ChkUnNum-ChkUnNum+1);
ChkIndZ(ChkInds(Chktrtr)) = StationUnNum*RGBImgNum+CamNum*CamUnNum+(Chktrtr*ChkUnNum);
end

Amat = zeros(RGBImgNum * length(PointData(1).XGr) * 3,StationUnNum*RGBImgNum+CamNum*CamUnNum+ChkNum*ChkUnNum);
Lmat = zeros(RGBImgNum * length(PointData(1).XGr) * 3,1);
CamDataHist(CamNum*CamUnNum,10)=0; CtrlHist(2,10)=0;
while loop
tic();
for rtrRGBImgNum=1:RGBImgNum 
PointId = Data(rtrRGBImgNum).PointId;
CamId = Data(rtrRGBImgNum).CamId;
StationId = Data(rtrRGBImgNum).StationId;
if ~StationData(StationId).active
    continue;
end

PointAvb = StationData(StationId).PointAvb;

fl = CamData(CamId).fvalue;
Kcoefs = CamData(CamId).Kcoefs;Pcoefs = CamData(CamId).Pcoefs;Bcoefs = CamData(CamId).Bcoefs;
BPK1 = Kcoefs(1);BPK2 = Kcoefs(2);BPK3 = Kcoefs(3);
BPK4 = Pcoefs(1);BPK5 = Pcoefs(2);BPK6 = Bcoefs(1);BPK7 = Bcoefs(2);
omega = StationData(StationId).omega;
phi = StationData(StationId).phi;
kappa = StationData(StationId).kappa;
XCntGr = StationData(StationId).Xcnt;
YCntGr = StationData(StationId).Ycnt;
ZCntGr = StationData(StationId).Zcnt;

XCoordGrMat = PointData(PointId).XGr;
YCoordGrMat = PointData(PointId).YGr;
ZCoordGrMat = PointData(PointId).ZGr;

Cntrl = PointData(PointId).Cntrl;
        
RGBPntNum=numel(XCoordGrMat);

trgxmat = Data(rtrRGBImgNum).trgxmat;
trgymat = Data(rtrRGBImgNum).trgymat;

ppcoord = CamData(CamId).ppcoord;
xCntImg = ppcoord(1);yCntImg = ppcoord(2);

ST = CamUnNum*CamId-CamUnNum+1:CamUnNum*CamId-0; 
CfC = ST(1);

CyCC = ST(end);

ST = (CamUnNum*CamNum)+StationUnNum*StationId-StationUnNum+1:...
    (CamUnNum*CamNum)+StationUnNum*StationId; 
SXC= ST(1);

SKC= ST(6);

for rtrRGBPntNum=1:RGBPntNum 
    if ~PointAvb(rtrRGBPntNum) || ~Cntrl(rtrRGBPntNum)
        if PointAvb(rtrRGBPntNum) && ~Cntrl(rtrRGBPntNum)
    ZCoordGr = ZCoordGrMat(rtrRGBPntNum);xCoordImg = trgxmat(rtrRGBPntNum);yCoordImg = trgymat(rtrRGBPntNum);
    CtrlHist(CamId*2-1,looprtr+1)=XCoordGrMat(rtrRGBPntNum) - eval_FRLCEX(BPK1,BPK2,BPK3,BPK4,BPK5,BPK6,BPK7,XCntGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi,xCntImg,xCoordImg,yCntImg,yCoordImg);
    CtrlHist(CamId*2-0,looprtr+1)=YCoordGrMat(rtrRGBPntNum) - eval_FRLCEY(BPK1,BPK2,BPK3,BPK4,BPK5,BPK6,BPK7,YCntGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi,xCntImg,xCoordImg,yCntImg,yCoordImg);
        else
            continue
        end
    end
    rowrtr = rowrtr + 1;

    XCoordGr = XCoordGrMat(rtrRGBPntNum);
    YCoordGr = YCoordGrMat(rtrRGBPntNum);
    ZCoordGr = ZCoordGrMat(rtrRGBPntNum);
    
    xCoordImg = trgxmat(rtrRGBPntNum);
    yCoordImg = trgymat(rtrRGBPntNum);
    
    
    Amat(rowrtr,CfC:CyCC)=...
        [eval_DfFDXfl(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,kappa,omega,phi),eval_DfFDXBPK1(xCntImg,xCoordImg,yCntImg,yCoordImg),...
        eval_DfFDXBPK2(xCntImg,xCoordImg,yCntImg,yCoordImg),eval_DfFDXBPK3(xCntImg,xCoordImg,yCntImg,yCoordImg),eval_DfFDXBPK4(xCntImg,xCoordImg,yCntImg,yCoordImg),eval_DfFDXBPK5(xCntImg,xCoordImg,yCntImg,yCoordImg),eval_DfFDXBPK6(xCntImg,xCoordImg),eval_DfFDXBPK7(yCntImg,yCoordImg)...
        eval_DfFDXxCntImg(BPK1,BPK2,BPK3,BPK4,BPK5,BPK6,xCntImg,xCoordImg,yCntImg,yCoordImg),eval_DfFDXyCntImg(BPK1,BPK2,BPK3,BPK4,BPK5,BPK7,xCntImg,xCoordImg,yCntImg,yCoordImg)...
        ];
    
    Amat(rowrtr,SXC:SKC)=...
        [eval_DfFDXXCntGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),...
        eval_DfFDXYCntGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),...
        eval_DfFDXZCntGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),...
        eval_DfFDXomega(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),...
        eval_DfFDXphi(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),...
        eval_DfFDXkappa(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)];
    
    Lmat(rowrtr,1) = eval_FDX(BPK1,BPK2,BPK3,BPK4,BPK5,BPK6,BPK7,XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi,xCntImg,xCoordImg,yCntImg,yCoordImg);
    if ~Cntrl(rtrRGBPntNum)
    Amat(rowrtr,ChkIndX(rtrRGBPntNum):ChkIndZ(rtrRGBPntNum))=...
        [eval_DfFDXXCoordGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),eval_DfFDXYCoordGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),eval_DfFDXZCoordGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),...
        ];
    end

    rowrtr=rowrtr+1;
    
    Amat(rowrtr,CfC:CyCC)=...
        [eval_DfFDYfl(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,kappa,omega,phi),...
        eval_DfFDYBPK1(xCntImg,xCoordImg,yCntImg,yCoordImg),eval_DfFDYBPK2(xCntImg,xCoordImg,yCntImg,yCoordImg),eval_DfFDYBPK3(xCntImg,xCoordImg,yCntImg,yCoordImg),...
        eval_DfFDYBPK4(xCntImg,xCoordImg,yCntImg,yCoordImg),eval_DfFDYBPK5(xCntImg,xCoordImg,yCntImg,yCoordImg),0,0,...
        eval_DfFDYxCntImg(BPK1,BPK2,BPK3,BPK4,BPK5,xCntImg,xCoordImg,yCntImg,yCoordImg),eval_DfFDYyCntImg(BPK1,BPK2,BPK3,BPK4,BPK5,xCntImg,xCoordImg,yCntImg,yCoordImg)...
        ];
    
    Amat(rowrtr,SXC:SKC)=...
        [eval_DfFDYXCntGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),...
        eval_DfFDYYCntGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),...
        eval_DfFDYZCntGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),...
        eval_DfFDYomega(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),...
        eval_DfFDYphi(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),...
        eval_DfFDYkappa(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)];
    
    Lmat(rowrtr,1) = eval_FDY(BPK1,BPK2,BPK3,BPK4,BPK5,XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi,xCntImg,xCoordImg,yCntImg,yCoordImg);
    if ~Cntrl(rtrRGBPntNum)
    Amat(rowrtr,ChkIndX(rtrRGBPntNum):ChkIndZ(rtrRGBPntNum))=...
        [eval_DfFDYXCoordGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),eval_DfFDYYCoordGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),eval_DfFDYZCoordGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi),...
        ];
    end
end
end
toc()
Amat = Amat(1:rowrtr,:);
Lmat = Lmat(1:rowrtr,:);
Xmat = Amat\Lmat;

    for StationId=1:size(StationData,2)
        ST = (CamUnNum*CamNum)+StationUnNum*StationId-StationUnNum+1:...
            (CamUnNum*CamNum)+StationUnNum*StationId; % StatoinValuesTemp
        SXC= ST(1);SYC= ST(2);SZC= ST(3);SOC= ST(4);SPC= ST(5);SKC= ST(6);
        
        StationData(StationId).omega = StationData(StationId).omega - Xmat(SOC);
        StationData(StationId).phi = StationData(StationId).phi - Xmat(SPC);
        StationData(StationId).kappa = StationData(StationId).kappa - Xmat(SKC);
        StationData(StationId).Xcnt = StationData(StationId).Xcnt - Xmat(SXC);
        StationData(StationId).Ycnt = StationData(StationId).Ycnt - Xmat(SYC);
        StationData(StationId).Zcnt = StationData(StationId).Zcnt - Xmat(SZC);
    end
    for CamId=1:CamNum
        ST = CamUnNum*CamId-CamUnNum+1:CamUnNum*CamId-0; % StatoinValuesTemp
        CfC = ST(1);CK1C = ST(2);CK2C = ST(3);CK3C = ST(4);CP1C=ST(5);CP2C= ST(6);CB1C= ST(7);CB2C =ST(8);CxCC =ST(9);CyCC =ST(10);
        CamData(CamId).fvalue=CamData(CamId).fvalue - Xmat(CfC);
        CamData(CamId).Kcoefs=...
            CamData(CamId).Kcoefs - [Xmat(CK1C),Xmat(CK2C),Xmat(CK3C)];
        CamData(CamId).Pcoefs=...
            CamData(CamId).Pcoefs - [Xmat(CP1C),Xmat(CP2C)];
        CamData(CamId).Bcoefs=...
            CamData(CamId).Bcoefs - [Xmat(CB1C),Xmat(CB2C)];
        CamData(CamId).ppcoord=...
            CamData(CamId).ppcoord- [Xmat(CxCC),Xmat(CyCC)];
    end
    for Chktrtr=1:ChkNum
        PointData(PointId).XGr(ChkInds(Chktrtr))=XCoordGrMat(ChkInds(Chktrtr)) - Xmat(ChkIndX(ChkInds(Chktrtr)));
        PointData(PointId).YGr(ChkInds(Chktrtr))=YCoordGrMat(ChkInds(Chktrtr)) - Xmat(ChkIndX(ChkInds(Chktrtr))+1);
        PointData(PointId).ZGr(ChkInds(Chktrtr))=ZCoordGrMat(ChkInds(Chktrtr)) - Xmat(ChkIndZ(ChkInds(Chktrtr)));
    end

    looprtr=looprtr+1;
    rowrtr=0;
    Xmatcell=[Xmatcell,Xmat];
    set(table1,'data',Xmatcell)
    if (max(abs(Xmat(:)))<.0001|| (looprtr>5))
        loop = false;
    end
end



%% -----------------FUNCTIONS----------------------------
% function out=eval_Mmat(kappa,omega,phi)
% out=matrix([[cos(kappa)*cos(phi), cos(omega)*sin(kappa) + cos(kappa)*sin(omega)*sin(phi), sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*sin(phi)], [-cos(phi)*sin(kappa), cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi), cos(kappa)*sin(omega) + cos(omega)*sin(kappa)*sin(phi)], [sin(phi), -cos(phi)*sin(omega), cos(omega)*cos(phi)]]);
function out=eval_FBrownx(BPK1,BPK2,BPK3,BPK4,BPK5,BPK6,BPK7,xCntImg,xCoordImg,yCntImg,yCoordImg)
out=BPK4*(3*(xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) - BPK7*(yCntImg - yCoordImg) - ...
    BPK6*(xCntImg - xCoordImg) - (xCntImg - xCoordImg)*...
    (BPK1*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) + BPK2*((xCntImg - xCoordImg)^2 + ...
    (yCntImg - yCoordImg)^2)^2 + BPK3*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3) + 2*BPK5*(xCntImg - xCoordImg)*(yCntImg - yCoordImg);
function out=eval_FBrowny(BPK1,BPK2,BPK3,BPK4,BPK5,xCntImg,xCoordImg,yCntImg,yCoordImg)
out=BPK5*((xCntImg - xCoordImg)^2 + 3*(yCntImg - yCoordImg)^2) - (yCntImg - yCoordImg)*...
    (BPK1*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) + BPK2*((xCntImg - xCoordImg)^2 + ...
    (yCntImg - yCoordImg)^2)^2 + BPK3*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3) +...
    2*BPK4*(xCntImg - xCoordImg)*(yCntImg - yCoordImg);
function out=eval_FRLCEX(BPK1,BPK2,BPK3,BPK4,BPK5,BPK6,BPK7,XCntGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi,xCntImg,xCoordImg,yCntImg,yCoordImg)
out=XCntGr - ((ZCntGr - ZCoordGr)*(fl*sin(phi) + cos(kappa)*cos(phi)*(xCntImg - xCoordImg + ...
    BPK6*(xCntImg - xCoordImg) + BPK7*(yCntImg - yCoordImg) - BPK4*(3*(xCntImg - xCoordImg)^2 + ...
    (yCntImg - yCoordImg)^2) + (xCntImg - xCoordImg)*(BPK1*((xCntImg - xCoordImg)^2 + (yCntImg - ...
    yCoordImg)^2) + BPK2*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2 + ...
    BPK3*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3) - ...
    2*BPK5*(xCntImg - xCoordImg)*(yCntImg - yCoordImg)) + ...
    cos(phi)*sin(kappa)*(yCoordImg - yCntImg + BPK5*((xCntImg - xCoordImg)^2 + 3*(yCntImg - yCoordImg)^2) - ...
    (yCntImg - yCoordImg)*(BPK1*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) + BPK2*((xCntImg - xCoordImg)^2 + ...
    (yCntImg - yCoordImg)^2)^2 + BPK3*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3) + 2*BPK4*(xCntImg - xCoordImg)...
    *(yCntImg - yCoordImg))))/((sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*sin(phi))*(xCntImg - xCoordImg + ...
    BPK6*(xCntImg - xCoordImg) + BPK7*(yCntImg - yCoordImg) - BPK4*(3*(xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) +...
    (xCntImg - xCoordImg)*(BPK1*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) + BPK2*((xCntImg - xCoordImg)^2 + ...
    (yCntImg - yCoordImg)^2)^2 + BPK3*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3) - 2*BPK5*(xCntImg - xCoordImg)*...
    (yCntImg - yCoordImg)) - (cos(kappa)*sin(omega) + cos(omega)*sin(kappa)*sin(phi))*(yCoordImg - yCntImg + BPK5*((xCntImg - ...
    xCoordImg)^2 + 3*(yCntImg - yCoordImg)^2) - (yCntImg - yCoordImg)*(BPK1*((xCntImg - xCoordImg)^2 + ...
    (yCntImg - yCoordImg)^2) + BPK2*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2 + BPK3*...
    ((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3) + 2*BPK4*(xCntImg - xCoordImg)*(yCntImg - yCoordImg)) + ...
    fl*cos(omega)*cos(phi));
function out=eval_FRLCEY(BPK1,BPK2,BPK3,BPK4,BPK5,BPK6,BPK7,YCntGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi,xCntImg,xCoordImg,yCntImg,yCoordImg)
out=YCntGr + ((ZCntGr - ZCoordGr)*((cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi))*(yCoordImg - yCntImg + ...
    BPK5*((xCntImg - xCoordImg)^2 + 3*(yCntImg - yCoordImg)^2) - (yCntImg - yCoordImg)*(BPK1*((xCntImg - xCoordImg)^2 + ...
    (yCntImg - yCoordImg)^2) + BPK2*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2 + BPK3*((xCntImg - xCoordImg)^2 + ...
    (yCntImg - yCoordImg)^2)^3) + 2*BPK4*(xCntImg - xCoordImg)*(yCntImg - yCoordImg)) - (cos(omega)*sin(kappa) + ...
    cos(kappa)*sin(omega)*sin(phi))*(xCntImg - xCoordImg + BPK6*(xCntImg - xCoordImg) + BPK7*(yCntImg - yCoordImg) -...
    BPK4*(3*(xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) + (xCntImg - xCoordImg)*(BPK1*((xCntImg - xCoordImg)^2 + ...
    (yCntImg - yCoordImg)^2) + BPK2*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2 + BPK3*((xCntImg - xCoordImg)^2 + ...
    (yCntImg - yCoordImg)^2)^3) - 2*BPK5*(xCntImg - xCoordImg)*(yCntImg - yCoordImg)) + ...
    fl*cos(phi)*sin(omega)))/((sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*sin(phi))*...
    (xCntImg - xCoordImg + BPK6*(xCntImg - xCoordImg) + BPK7*(yCntImg - yCoordImg) - BPK4*(3*(xCntImg - xCoordImg)^2 + ...
    (yCntImg - yCoordImg)^2) + (xCntImg - xCoordImg)*(BPK1*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) + BPK2*...
    ((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2 + BPK3*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3) - ...
    2*BPK5*(xCntImg - xCoordImg)*(yCntImg - yCoordImg)) - (cos(kappa)*sin(omega) + cos(omega)*sin(kappa)*sin(phi))*...
    (yCoordImg - yCntImg + BPK5*((xCntImg - xCoordImg)^2 + 3*(yCntImg - yCoordImg)^2) - (yCntImg - yCoordImg)*...
    (BPK1*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) + BPK2*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2 + ...
    BPK3*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3) + 2*BPK4*(xCntImg - xCoordImg)*(yCntImg - yCoordImg)) + ...
    fl*cos(omega)*cos(phi));

function out=eval_DfFDXfl(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,kappa,omega,phi)
out=((cos(omega)*sin(kappa) + cos(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (sin(kappa)*sin(omega) - ...
    cos(kappa)*cos(omega)*sin(phi))*(ZCntGr - ZCoordGr) + cos(kappa)*cos(phi)*(XCntGr - XCoordGr))/(sin(phi)*...
    (XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr));

function out=eval_DfFDXBPK1(xCntImg,xCoordImg,yCntImg,yCoordImg)
out=-((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)*(xCntImg - xCoordImg);

function out=eval_DfFDXBPK2(xCntImg,xCoordImg,yCntImg,yCoordImg)
out=-((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2*(xCntImg - xCoordImg);

function out=eval_DfFDXBPK3(xCntImg,xCoordImg,yCntImg,yCoordImg)
out=-((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3*(xCntImg - xCoordImg);

function out=eval_DfFDXXCntGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=(fl*cos(kappa)*cos(phi))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - ...
    cos(phi)*sin(omega)*(YCntGr - YCoordGr)) - (fl*sin(phi)*((cos(omega)*sin(kappa) + cos(kappa)*sin(omega)*...
    sin(phi))*(YCntGr - YCoordGr) + (sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*sin(phi))*(ZCntGr - ZCoordGr) + ...
    cos(kappa)*cos(phi)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - ...
    cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2;

function out=eval_DfFDXYCntGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=(fl*(cos(omega)*sin(kappa) + cos(kappa)*sin(omega)*sin(phi)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr)) + (fl*cos(phi)*sin(omega)*((cos(omega)*sin(kappa) + ...
    cos(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*sin(phi))*...
    (ZCntGr - ZCoordGr) + cos(kappa)*cos(phi)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2;

function out=eval_DfFDXZCntGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=(fl*(sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*sin(phi)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr)) - (fl*cos(omega)*cos(phi)*((cos(omega)*sin(kappa) + ...
    cos(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*sin(phi))*...
    (ZCntGr - ZCoordGr) + cos(kappa)*cos(phi)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2;

function out=eval_DfFDXomega(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=(fl*(cos(omega)*cos(phi)*(YCntGr - YCoordGr) + cos(phi)*sin(omega)*(ZCntGr - ZCoordGr))*((cos(omega)*sin(kappa) + ...
    cos(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*sin(phi))*...
    (ZCntGr - ZCoordGr) + cos(kappa)*cos(phi)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2 - (fl*((sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*...
    sin(phi))*(YCntGr - YCoordGr) - (cos(omega)*sin(kappa) + cos(kappa)*sin(omega)*sin(phi))*(ZCntGr - ZCoordGr)))/(sin(phi)*...
    (XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr));

function out=eval_DfFDXphi(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=- (fl*(cos(kappa)*sin(phi)*(XCntGr - XCoordGr) + cos(kappa)*cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - ...
    cos(kappa)*cos(phi)*sin(omega)*(YCntGr - YCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr)) - (fl*(cos(phi)*(XCntGr - XCoordGr) - cos(omega)*...
    sin(phi)*(ZCntGr - ZCoordGr) + sin(omega)*sin(phi)*(YCntGr - YCoordGr))*((cos(omega)*sin(kappa) + cos(kappa)*...
    sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*sin(phi))*(ZCntGr - ZCoordGr) + ...
    cos(kappa)*cos(phi)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) -...
    cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2;

function out=eval_DfFDXkappa(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=(fl*((cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (cos(kappa)*sin(omega) + ...
    cos(omega)*sin(kappa)*sin(phi))*(ZCntGr - ZCoordGr) - cos(phi)*sin(kappa)*(XCntGr - XCoordGr)))/(sin(phi)*...
    (XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr));

function out=eval_FDX(BPK1,BPK2,BPK3,BPK4,BPK5,BPK6,BPK7,XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi,xCntImg,xCoordImg,yCntImg,yCoordImg)
out=xCoordImg - xCntImg - BPK6*(xCntImg - xCoordImg) - BPK7*(yCntImg - yCoordImg) + BPK4*(3*(xCntImg - xCoordImg)^2 + ...
    (yCntImg - yCoordImg)^2) - (xCntImg - xCoordImg)*(BPK1*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) + BPK2*...
    ((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2 + BPK3*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3) + ...
    2*BPK5*(xCntImg - xCoordImg)*(yCntImg - yCoordImg) + (fl*((cos(omega)*sin(kappa) + cos(kappa)*sin(omega)*sin(phi))*...
    (YCntGr - YCoordGr) + (sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*sin(phi))*(ZCntGr - ZCoordGr) + cos(kappa)*cos(phi)*...
    (XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*...
    (YCntGr - YCoordGr));

function out=eval_DfFDXXCoordGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=(fl*sin(phi)*((cos(omega)*sin(kappa) + cos(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (sin(kappa)*sin(omega) - ...
    cos(kappa)*cos(omega)*sin(phi))*(ZCntGr - ZCoordGr) + cos(kappa)*cos(phi)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - ...
    XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2 - (fl*cos(kappa)*...
    cos(phi))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr));

function out=eval_DfFDXYCoordGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=- (fl*(cos(omega)*sin(kappa) + cos(kappa)*sin(omega)*sin(phi)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr)) - (fl*cos(phi)*sin(omega)*((cos(omega)*sin(kappa) + ...
    cos(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*sin(phi))*...
    (ZCntGr - ZCoordGr) + cos(kappa)*cos(phi)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2;

function out=eval_DfFDXZCoordGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=(fl*cos(omega)*cos(phi)*((cos(omega)*sin(kappa) + cos(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (sin(kappa)*...
    sin(omega) - cos(kappa)*cos(omega)*sin(phi))*(ZCntGr - ZCoordGr) + cos(kappa)*cos(phi)*(XCntGr - XCoordGr)))/(sin(phi)*...
    (XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2 - (fl*(sin(kappa)*...
    sin(omega) - cos(kappa)*cos(omega)*sin(phi)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) -...
    cos(phi)*sin(omega)*(YCntGr - YCoordGr));

function out=eval_DfFDXBPK4(xCntImg,xCoordImg,yCntImg,yCoordImg)
out=3*(xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2;

function out=eval_DfFDXBPK5(xCntImg,xCoordImg,yCntImg,yCoordImg)
out=2*(xCntImg - xCoordImg)*(yCntImg - yCoordImg);

function out=eval_DfFDXBPK6(xCntImg,xCoordImg)
out=xCoordImg - xCntImg;

function out=eval_DfFDXBPK7(yCntImg,yCoordImg)
out=yCoordImg - yCntImg;

function out=eval_DfFDXxCntImg(BPK1,BPK2,BPK3,BPK4,BPK5,BPK6,xCntImg,xCoordImg,yCntImg,yCoordImg)
out=2*BPK5*(yCntImg - yCoordImg) - BPK1*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) - BPK6 - BPK2*((xCntImg - ...
    xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2 - BPK3*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3 + BPK4*(6*xCntImg -...
    6*xCoordImg) - (xCntImg - xCoordImg)*(BPK1*(2*xCntImg - 2*xCoordImg) + 2*BPK2*(2*xCntImg - 2*xCoordImg)*((xCntImg - ...
    xCoordImg)^2 + (yCntImg - yCoordImg)^2) + 3*BPK3*(2*xCntImg - 2*xCoordImg)*((xCntImg - xCoordImg)^2 + (yCntImg - ...
    yCoordImg)^2)^2) - 1;

function out=eval_DfFDXyCntImg(BPK1,BPK2,BPK3,BPK4,BPK5,BPK7,xCntImg,xCoordImg,yCntImg,yCoordImg)
out=2*BPK5*(xCntImg - xCoordImg) - BPK7 + BPK4*(2*yCntImg - 2*yCoordImg) - (xCntImg - xCoordImg)*(BPK1*(2*yCntImg - ...
    2*yCoordImg) + 2*BPK2*(2*yCntImg - 2*yCoordImg)*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) + 3*BPK3*...
    (2*yCntImg - 2*yCoordImg)*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2);

function out=eval_DfFDYfl(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,kappa,omega,phi)
out=((cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (cos(kappa)*sin(omega) + cos(omega)*...
    sin(kappa)*sin(phi))*(ZCntGr - ZCoordGr) - cos(phi)*sin(kappa)*(XCntGr - XCoordGr))/(sin(phi)*(XCntGr - XCoordGr) + ...
    cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr));

function out=eval_DfFDYBPK1(xCntImg,xCoordImg,yCntImg,yCoordImg)
out=-((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)*(yCntImg - yCoordImg);

function out=eval_DfFDYBPK2(xCntImg,xCoordImg,yCntImg,yCoordImg)
out=-((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2*(yCntImg - yCoordImg);

function out=eval_DfFDYBPK3(xCntImg,xCoordImg,yCntImg,yCoordImg)
out=-((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3*(yCntImg - yCoordImg);

function out=eval_DfFDYXCntGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=- (fl*sin(phi)*((cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (cos(kappa)*sin(omega) + ...
    cos(omega)*sin(kappa)*sin(phi))*(ZCntGr - ZCoordGr) - cos(phi)*sin(kappa)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - ...
    XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2 - (fl*cos(phi)*...
    sin(kappa))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - ...
    YCoordGr));

function out=eval_DfFDYYCntGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=(fl*(cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr)) + (fl*cos(phi)*sin(omega)*((cos(kappa)*cos(omega) - ...
    sin(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (cos(kappa)*sin(omega) + cos(omega)*sin(kappa)*sin(phi))*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(kappa)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2;

function out=eval_DfFDYZCntGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=(fl*(cos(kappa)*sin(omega) + cos(omega)*sin(kappa)*sin(phi)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr)) - (fl*cos(omega)*cos(phi)*((cos(kappa)*cos(omega) - ...
    sin(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (cos(kappa)*sin(omega) + cos(omega)*sin(kappa)*sin(phi))*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(kappa)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2;

function out=eval_DfFDYomega(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=(fl*(cos(omega)*cos(phi)*(YCntGr - YCoordGr) + cos(phi)*sin(omega)*(ZCntGr - ZCoordGr))*((cos(kappa)*cos(omega) - ...
    sin(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (cos(kappa)*sin(omega) + cos(omega)*sin(kappa)*sin(phi))*(ZCntGr - ...
    ZCoordGr) - cos(phi)*sin(kappa)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ...
    ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2 - (fl*((cos(kappa)*sin(omega) + cos(omega)*sin(kappa)*sin(phi))*...
    (YCntGr - YCoordGr) - (cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi))*(ZCntGr - ZCoordGr)))/(sin(phi)*(XCntGr - ...
    XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr));

function out=eval_DfFDYphi(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=(fl*(sin(kappa)*sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*sin(kappa)*(ZCntGr - ZCoordGr) - cos(phi)*sin(kappa)*...
    sin(omega)*(YCntGr - YCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*...
    sin(omega)*(YCntGr - YCoordGr)) - (fl*(cos(phi)*(XCntGr - XCoordGr) - cos(omega)*sin(phi)*(ZCntGr - ZCoordGr) + ...
    sin(omega)*sin(phi)*(YCntGr - YCoordGr))*((cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + ...
    (cos(kappa)*sin(omega) + cos(omega)*sin(kappa)*sin(phi))*(ZCntGr - ZCoordGr) - cos(phi)*sin(kappa)*(XCntGr - XCoordGr)))/...
    (sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2;

function out=eval_DfFDYkappa(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=-(fl*((cos(omega)*sin(kappa) + cos(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (sin(kappa)*sin(omega) - cos(kappa)...
    *cos(omega)*sin(phi))*(ZCntGr - ZCoordGr) + cos(kappa)*cos(phi)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + ...
    cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr));

function out=eval_FDY(BPK1,BPK2,BPK3,BPK4,BPK5,XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi,xCntImg,xCoordImg,yCntImg,yCoordImg)
out=yCoordImg - yCntImg + BPK5*((xCntImg - xCoordImg)^2 + 3*(yCntImg - yCoordImg)^2) - (yCntImg - yCoordImg)*(BPK1*...
    ((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) + BPK2*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2 + ...
    BPK3*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3) + 2*BPK4*(xCntImg - xCoordImg)*(yCntImg - yCoordImg) + ...
    (fl*((cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (cos(kappa)*sin(omega) + cos(omega)*...
    sin(kappa)*sin(phi))*(ZCntGr - ZCoordGr) - cos(phi)*sin(kappa)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) +...
    cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr));

function out=eval_DfFDYXCoordGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=(fl*sin(phi)*((cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (cos(kappa)*sin(omega) + ...
    cos(omega)*sin(kappa)*sin(phi))*(ZCntGr - ZCoordGr) - cos(phi)*sin(kappa)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - ...
    XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2 + (fl*cos(phi)*...
    sin(kappa))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - ...
    YCoordGr));

function out=eval_DfFDYYCoordGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=- (fl*(cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*...
    (ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr)) - (fl*cos(phi)*sin(omega)*((cos(kappa)*cos(omega) - ...
    sin(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + (cos(kappa)*sin(omega) + cos(omega)*sin(kappa)*sin(phi))*(ZCntGr - ...
    ZCoordGr) - cos(phi)*sin(kappa)*(XCntGr - XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ...
    ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr))^2;

function out=eval_DfFDYZCoordGr(XCntGr,XCoordGr,YCntGr,YCoordGr,ZCntGr,ZCoordGr,fl,kappa,omega,phi)
out=(fl*cos(omega)*cos(phi)*((cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi))*(YCntGr - YCoordGr) + ...
    (cos(kappa)*sin(omega) + cos(omega)*sin(kappa)*sin(phi))*(ZCntGr - ZCoordGr) - cos(phi)*sin(kappa)*(XCntGr - ...
    XCoordGr)))/(sin(phi)*(XCntGr - XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*...
    (YCntGr - YCoordGr))^2 - (fl*(cos(kappa)*sin(omega) + cos(omega)*sin(kappa)*sin(phi)))/(sin(phi)*(XCntGr - ...
    XCoordGr) + cos(omega)*cos(phi)*(ZCntGr - ZCoordGr) - cos(phi)*sin(omega)*(YCntGr - YCoordGr));

function out=eval_DfFDYBPK4(xCntImg,xCoordImg,yCntImg,yCoordImg)
out=2*(xCntImg - xCoordImg)*(yCntImg - yCoordImg);

function out=eval_DfFDYBPK5(xCntImg,xCoordImg,yCntImg,yCoordImg)
out=(xCntImg - xCoordImg)^2 + 3*(yCntImg - yCoordImg)^2;


function out=eval_DfFDYxCntImg(BPK1,BPK2,BPK3,BPK4,BPK5,xCntImg,xCoordImg,yCntImg,yCoordImg)
out=2*BPK4*(yCntImg - yCoordImg) + BPK5*(2*xCntImg - 2*xCoordImg) - (yCntImg - yCoordImg)*(BPK1*(2*xCntImg - 2*xCoordImg) ...
    + 2*BPK2*(2*xCntImg - 2*xCoordImg)*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) + 3*BPK3*(2*xCntImg - 2*xCoordImg)...
    *...
    ((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2);

function out=eval_DfFDYyCntImg(BPK1,BPK2,BPK3,BPK4,BPK5,xCntImg,xCoordImg,yCntImg,yCoordImg)
out=2*BPK4*(xCntImg - xCoordImg) - BPK1*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2) - BPK2*((xCntImg - xCoordImg)^2 +...
    (yCntImg - yCoordImg)^2)^2 - BPK3*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^3 + BPK5*(6*yCntImg - 6*yCoordImg)...
    - (yCntImg - yCoordImg)*(BPK1*(2*yCntImg - 2*yCoordImg) + 2*BPK2*(2*yCntImg - 2*yCoordImg)*((xCntImg - xCoordImg)^2 +...
    (yCntImg - yCoordImg)^2) + 3*BPK3*(2*yCntImg - 2*yCoordImg)*((xCntImg - xCoordImg)^2 + (yCntImg - yCoordImg)^2)^2) - 1;