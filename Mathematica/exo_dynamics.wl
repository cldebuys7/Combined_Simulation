(* ::Package:: *)

(* ::Section:: *)
(*Package Initializations*)


(* ::Subsection::Closed:: *)
(*Include*)


SetDirectory[NotebookDirectory[]]
UtilPath=FileNameJoin[{Directory[],"utils"}]
$Path=DeleteDuplicates[Append[$Path,UtilPath]]
SetDirectory[UtilPath]


Needs["Units`"];
Needs["RobotLinks`"];
Needs["ToMatlab`"];
Needs["ExtraUtil`"];
Needs["ToCpp`"];
On[Assert];


(* ::Subsection:: *)
(*User-defined functions*)


Vec::usage="Vec[x] Turn arbitary list into vector";
Vec[x_]:=Transpose[{Flatten[x]}];
ToExpressionEx::usage="ToExpressionEx[expr]  loosely converts any string types in an 0- to n-dimensional list to an expression.";
ToExpressionEx[value_]:=Module[{result},result=If[StringQ[value],ToExpression[value],If[ListQ[value],Map[If[StringQ[#],ToExpression[#],#]&,value,-1],value]];
Return[result];];
RationalizeEx::usage="RationalizeEx[expr]  loosely rationalize any expression to an arbitrary precision";
RationalizeEx[expr_]:=Rationalize[expr,0];
RationalizeEx[expr_List]:=Map[RationalizeEx,expr,-1];
RationalizeAny::usage="RationalizeAny[value]  convert `value` to an expression and use RationalizeEx";
RationalizeAny[expr_]:=RationalizeEx[ToExpressionEx[expr]];
(*From:http://mathworld.wolfram.com/BlockDiagonalMatrix.html*)
BlockDiagonalMatrix[b:{__?MatrixQ}]:=Module[{r,c,n=Length[b],i,j},
{r,c}=Transpose[Dimensions/@b];
ArrayFlatten[
Table[
If[i==j,b[[i]],ConstantArray[0,{r[[i]],c[[j]]}]]
,
{i,n},{j,n}
]
]
];
EnsureDirectoryExists[dir_?StringQ]:=Module[{Pieces,cur},
Pieces=FileNameSplit[dir];
Table[
cur=FileNameJoin[Pieces[[1;;i]]];
If[!DirectoryQ[cur],
CreateDirectory[cur];
];
,
{i,Length[Pieces]}
];
];

ParallelSimplify[A_?MatrixQ]:=ParallelTable[Simplify[A[[i,j]]],{i,Dimensions[A][[1]]},{j,Dimensions[A][[2]]}]
ParallelSimplify[A_?VectorQ]:=ParallelTable[Simplify[A[[i]]],{i,Length[A]}]
ParallelSimplify[A_]:=Simplify[A]


GetBasisX[angle_]:=RotationMatrix[angle,{0,0,1}][[All,1]];
GetBasisY[angle_]:=RotationMatrix[angle,{0,0,1}][[All,2]];
GetBasisZ[angle_]:=RotationMatrix[angle,{0,0,1}][[All,3]];
RotX4[angle_]:=RotationMatrix[angle,{{0,1,0,0},{0,0,1,0}}];
RotY4[angle_]:=RotationMatrix[angle,{{0,0,1,0},{1,0,0,0}}];
RotZ4[angle_]:=RotationMatrix[angle,{{1,0,0,0},{0,1,0,0}}];
Trans4[dp_]:={{1,0,0,dp[[1]]},{0,1,0,dp[[2]]},{0,0,1,dp[[3]]},{0,0,0,1}};
GetVelTwoPt[vq_,w_,r_]:=vq+Cross[w,r];
GetAccTwoPt[aq_,alpha_,w_,r_]:=aq+Cross[alpha,r]+Cross[w,Cross[w,r]];
GetCComp[eq_,var_]:=If[D[D[eq,var],var]===0,D[eq,var],D[D[eq,var],var] var/2];
GetMMatrix[eq_,var_]:=Table[D[eq[[i]],var[[j]]],{i,Length[var]},{j,Length[var]}];
GetCMatrix[eq_,var_]:=Table[D[eq[[i]],var[[j]]]/2,{i,Length[var]},{j,Length[var]}];
GetGMatrix[eq_,var_]:=Table[eq[[i]],{i,Length[eq]}]/.Table[D[var[[i]],t]-> 0,{i,Length[var]}]/.Table[D[D[var[[i]],t],t]-> 0,{i,Length[var]}];


(* ::Section:: *)
(*Constants*)


g = 9.81;


(* ::Subsection:: *)
(*Degrees of freedom*)


ndof = 5; (* 4 joints *)
nb = 2; (*signifies the x and y position of the hip*)


(* ::Subsection:: *)
(*Lengths, Masses, and Center of Masses*)


mTotal = 8;
LimbAng = 120*Pi/180;


constsubs = {
a1 -> 0.3, b1 -> 0.2, mTorso -> 1,
a3 -> 0.22, b3 -> 0.22, mShank -> 0.5*0.25*mTotal,
a2 -> 0.30-0.05, b2 -> 0.2, mThigh -> 0.5*0.75*mTotal,
dStrap -> 0.3
}


masses ={mTorso,mThigh,mShank,mThigh,mShank} /.constsubs


(* ::Section:: *)
(*Forward Kinematics (configuration A)*)


(* ::Subsection:: *)
(*Joints(Generalized coordinates)*)


(* ::Subsubsection:: *)
(*Positions*)


q = Table[Subscript[\[Theta], i][t],{i,ndof}] (*joint variables*)
dq = D[q,t]
ddq = D[dq,t]
qb = {Subscript[r, x][t],Subscript[r, y][t]} (*position of stance heel*)
dqb = D[qb,t]
ddqb = D[dqb,t]


qe = Join[qb,q]
dqe = Join[dqb,dq]
ddqe = Join[ddqb,ddq]


(*St - Stance and Sw - Swing*)
Hip = {qb[[1]], qb[[2]], 0}; (*Stance foot*)

(* torso *)
temp = RotZ4[q[[1]]].Trans4[{a1,0,0}]; (*transformation matrix*)
TorsoCOM = temp[[1;;3, 4]] + Hip; (*takes the 1st - 3rd element of the 4th column of the transformation matrix and adds it to the Hip coordinates*)

temp = RotZ4[q[[1]]].Trans4[{a1+b1,0,0}]; (*transformation matrix*)
Shoulder = temp[[1;;3, 4]] + Hip; (*takes the 1st - 3rd element of the 4th column of the transformation matrix and adds it to the Hip coordinates*)

(* location of the torso restraint *)
temp = RotZ4[q[[1]]].Trans4[{dStrap,0,0}]; (*transformation matrix*)
Strap = temp[[1;;3, 4]] + Hip;

(* stance leg *)
temp = RotZ4[q[[2]]].Trans4[{b2,0,0}]; (*transformation matrix*)
StThighCOM = temp[[1;;3, 4]] + Hip; (*takes the 1st - 3rd element of the 4th column of the transformation matrix and adds it to the Hip coordinates*)

temp = RotZ4[q[[2]]].Trans4[{a2+b2+q[[3]],0,0}]; 
StKnee = temp[[1;;3, 4]] + Hip; 

temp = RotZ4[q[[2]]].RotZ4[Pi-LimbAng].Trans4[{b3,0,0}]; 
StShankCOM = temp[[1;;3, 4]] + StKnee; 

temp = RotZ4[q[[2]]].RotZ4[Pi-LimbAng].Trans4[{a3+b3,0,0}]; 
StFoot = temp[[1;;3, 4]] + StKnee //Simplify (*expression of rx and ry in terms of px and py*)

(* swing leg *)
temp = RotZ4[q[[4]]].Trans4[{b2,0,0}]; 
SwThighCOM = temp[[1;;3, 4]] + Hip ;

temp = RotZ4[q[[4]]].Trans4[{a2+b2+q[[5]],0,0}]; 
SwKnee = temp[[1;;3, 4]] + Hip;

temp = RotZ4[q[[4]]].RotZ4[Pi-LimbAng].Trans4[{b3,0,0}]; 
SwShankCOM = temp[[1;;3, 4]] + SwKnee;

temp = RotZ4[q[[4]]].RotZ4[Pi-LimbAng].Trans4[{a3+b3,0,0}]; 
SwFoot = temp[[1;;3, 4]] + SwKnee;

qCOM = {TorsoCOM, StThighCOM, StShankCOM,  SwThighCOM, SwShankCOM}/.constsubs; (*gives a matrix for all COM positions. Each row corresponds to a position vector*)

qEnds = {Hip, StKnee, StFoot, SwKnee, SwFoot, Shoulder}/.constsubs; (*gives a matrix for all joint positions and link ends. Each row corresponds to a position vector*)


(* ::Subsubsection:: *)
(*Velocities*)


(*Angular velocities*)
\[Omega]Torso = {0,0,dq[[1]]};
\[Omega]StShank = {0,0,dq[[2]]};
\[Omega]StThigh = {0,0,dq[[2]]};
\[Omega]SwThigh = {0,0,dq[[4]]};
\[Omega]SwShank = {0,0,dq[[4]]};

\[Omega] = {\[Omega]Torso, \[Omega]StThigh, \[Omega]StShank, \[Omega]SwThigh, \[Omega]SwShank}; (*Array with all angular velocities*)


(*linear velocities*)
vCOM = D[qCOM, t] //Simplify;  (*gives a matrix for all COM velocities. Each row corresponds to a velocity vector*)
vEnds = D[qEnds, t] //Simplify;  (*gives a matrix of joint and link end velocities. Each row corresponds to a velocity vector*)


(*linear accelerations*)
aEnds = D[vEnds,t] //Simplify;


(* ::Section:: *)
(*Equations of motion*)


(* ::Subsection:: *)
(*Inertias*)


IShank = 0.25*mShank*0.015^2 + 1/12*mShank*a3^2; (*shank is a rod of radius R=0.015. Nammi did something stupid. Correct it pleaseeee*)
IThighSt = 0.25*(0.5*mThigh)*0.02^2 + 1/3*(0.5*mThigh)*b2^2 + 0.25*(0.25*mThigh)*(0.02^2+0.015^2)+ 0.25*(0.25*mThigh)*0.015^2 + 1/3*(0.25*mThigh)*0.25^2 + 1/3*(0.25*mThigh)*a2^2 + (0.25*mThigh)*q[[2]]^2;
IThighSw = 0.25*(0.5*mThigh)*0.02^2 + 1/3*(0.5*mThigh)*b2^2 + 0.25*(0.25*mThigh)*(0.02^2+0.015^2)+ 0.25*(0.25*mThigh)*0.015^2 + 1/3*(0.25*mThigh)*0.25^2 + 1/3*(0.25*mThigh)*a2^2 + (0.25*mThigh)*q[[4]]^2;
ITorso = 1/3*mTorso*(a1+b1)^2;
Iz = {ITorso, IThighSt, IShank, IThighSw, IShank} /.constsubs (*array with all of the inertias*)


(* ::Subsection:: *)
(*Kinetic Energy*)


K = 1/2 Sum[masses[[i]] Dot[vCOM[[i]]//Flatten, vCOM[[i]]//Flatten], {i,1,ndof}]+ 1/2 Sum[Iz[[i]]*Dot[\[Omega][[i]],\[Omega][[i]]], {i,1,ndof}] /.constsubs  //Simplify;


(* ::Subsection:: *)
(*Potential Energy*)


P = Sum[masses[[i]]*g*qCOM[[i,2]], {i,1,ndof}];


(* ::Subsection:: *)
(*Equations of Motion*)


L = K - P;


eqList = Table[D[D[L,dqe[[i]]],t]-D[L,qe[[i]]],{i,nb+ndof}];


Dimensions[eqList]


Mmat = GetMMatrix[eqList,ddqe]//Simplify;


Cmat = InertiaToCoriolis[Mmat,qe,dqe]//Simplify;


Gvec = GetGMatrix[eqList,qe];


(* ::Section:: *)
(*Constraints*)


(* J1, J2, and J3 give the locations of the reaction forces from the exoskeleton *)
J1 = D[{Hip},{qe}];
J2 = D[{StFoot}//Flatten, {qe}];
J3 = D[{SwFoot}//Flatten,{qe}];
(* J4 ensures the torso position at the strap location is the same as that of the human *)
(*J4 = D[{Strap}//Flatten,{qe}];*)
J4 = D[{q[[1]]}//Flatten, {qe}];


J1dot = D[J1,t];
J2dot = D[J2,t];
J3dot = D[J3,t];
J4dot = D[J4,t];


(* ::Section:: *)
(*Output Jacobians*)


op = {qe[[4]],qe[[5]],qe[[6]],qe[[7]]};


xState = Join[qe,dqe];
Jo = D[op,{xState}];
dxState = Join[dqe,ddqe];
DJo = D[Jo.dxState,{xState}];


(* ::Section:: *)
(*Export to Matlab*)


statesubs=Flatten[Join[Table[{qe[[i]]->x[i],dqe[[i]]->x[nb+ndof+i]},{i,nb+ndof}]]];


generalFormatter[expr_]:=N[expr/.constsubs]/.statesubs;
SetOptions[WriteMatlabFunction,
Formatter->generalFormatter,
Directory->"expr_exo/model",
Arguments->{x}
];


SetDirectory[NotebookDirectory[]]
EnsureDirectoryExists["expr_exo/model"];


WriteMatlabFunction["Mmat_e",Mmat];
WriteMatlabFunction["Cmat_e",Cmat];
WriteMatlabFunction["Gvec_e",Gvec];


WriteMatlabFunction["Jhip_e",J1];
WriteMatlabFunction["JstFoot_e",J2];
WriteMatlabFunction["JswFoot_e",J3];
WriteMatlabFunction["Jtorso_e",J4];
WriteMatlabFunction["Jhipdot_e",J1dot];
WriteMatlabFunction["JstFootdot_e",J2dot];
WriteMatlabFunction["JswFootdot_e",J3dot];
WriteMatlabFunction["Jtorsodot_e",J4dot];


WriteMatlabFunction["qCOM_e",qCOM];
WriteMatlabFunction["qEnds_e",qEnds];
WriteMatlabFunction["vCOM_e",vCOM];
WriteMatlabFunction["vEnds_e",vEnds];
WriteMatlabFunction["aEnds_e",aEnds];


WriteMatlabFunction["Jo_e",Jo];
WriteMatlabFunction["DJo_e",DJo];
