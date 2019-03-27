(* ::Package:: *)

(* ::Section::Closed:: *)
(*Package Initializations*)


(* ::Subsection:: *)
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


(* ::Subsection::Closed:: *)
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
EnsureDirectoryExists[dir_?StringQ]:=Module[{pieces,cur},
pieces=FileNameSplit[dir];
Table[
cur=FileNameJoin[pieces[[1;;i]]];
If[!DirectoryQ[cur],
CreateDirectory[cur];
];
,
{i,Length[pieces]}
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


(* These are the dynamics for a kneed biped (2 hip joints, 2 knee joints). Absolute joint angles are used to simplify the matrices. *)
ndof = 5; (*    q1: torso angle   | origin-hip
				q2: stThigh angle | origin-hip
				q3: stShank angle | origin-knee				
				q4: swThigh angle | origin-hip
				q5: swShank angle | origin-knee     *)
nb = 2; (* signifies the x and y position of the hip *)


(* ::Subsection:: *)
(*Lengths, Masses, and Center of Masses*)


hTotal = 1.8034;
mTotal = 72.6;


constsubs = {
a3 -> 0.567*0.246*hTotal, b3 -> 0.433*0.246*hTotal, mShank -> 0.0465*mTotal,
a2 -> 0.567*0.272*hTotal, b2 -> 0.433*0.272*hTotal, mThigh -> 0.1*mTotal,
a1 -> 0.626*0.288*hTotal, b1 -> 0.374*0.288*hTotal, mTorso -> 0.678*mTotal,
dStrap -> 0.3
}

(*  a3 -> ankle to shankCOM, b3 -> knee to shankCOM, mShank -> 0.0465*mTotal,
	a2 -> knee to thighCOM, b2 -> hip to thighCOM, mThigh -> 0.1*mTotal,
	a1 -> hip to torsoCOM,  b1 -> shoulder to torsoCOM mTorso -> 0.678*mTotal                             *)


masses ={mTorso,mThigh,mShank,mThigh,mShank}/.constsubs
mass = Total[masses]/.constsubs;


(* ::Subsection:: *)
(*Inertias*)


gyr = {0.496*0.288*hTotal, 0.323*0.272*hTotal, 0.302*0.246*hTotal, 0.323*0.272*hTotal,  0.302*0.246*hTotal} (*Radius of gyration*)
Iz = masses*gyr^2 (*array with all of the inertias*)


(* ::Section:: *)
(*Forward Kinematics*)


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
Hip = {qb[[1]], qb[[2]], 0}; (* Hip Position *)

(* torso *)
temp = RotZ4[q[[1]]].Trans4[{a1,0,0}];
TorsoCOM = temp[[1;;3, 4]] + Hip;

temp = RotZ4[q[[1]]].Trans4[{a1+b1,0,0}];
Shoulder = temp[[1;;3, 4]] + Hip;

(* location of the torso restraint *)
temp = RotZ4[q[[1]]].Trans4[{dStrap,0,0}];
Strap = temp[[1;;3, 4]] + Hip;

(* stance leg *)
temp = RotZ4[q[[2]]].Trans4[{b2,0,0}];
StThighCOM = temp[[1;;3, 4]] + Hip;

temp = RotZ4[q[[2]]].Trans4[{a2+b2,0,0}];
StKnee = temp[[1;;3, 4]] + Hip;

temp = RotZ4[q[[3]]].Trans4[{b3,0,0}]; 
StShankCOM = temp[[1;;3, 4]] + StKnee; 

temp = RotZ4[q[[3]]].Trans4[{a3+b3,0,0}];
StFoot = temp[[1;;3, 4]] + StKnee;

(* swing leg *)
temp = RotZ4[q[[4]]].Trans4[{b2,0,0}];
SwThighCOM = temp[[1;;3, 4]] + Hip;

temp = RotZ4[q[[4]]].Trans4[{a2+b2,0,0}];
SwKnee = temp[[1;;3, 4]] + Hip;

temp = RotZ4[q[[5]]].Trans4[{b3,0,0}]; 
SwShankCOM = temp[[1;;3, 4]] + SwKnee; 

temp = RotZ4[q[[5]]].Trans4[{a3+b3,0,0}];
SwFoot = temp[[1;;3, 4]] + SwKnee;


qCOM = {TorsoCOM, StThighCOM, StShankCOM, SwThighCOM, SwShankCOM}; (*gives a matrix for all COM positions. Each row corresponds to a position vector*)
qEnds= {Hip, StKnee, StFoot, SwKnee, SwFoot, Shoulder}; (*gives a matrix for all joint positions and link ends. Each row corresponds to a position vector*)
Dimensions[qEnds]


(* ::Subsubsection:: *)
(*Velocities*)


(*Angular velocities*)
\[Omega]Torso = {0,0,dq[[1]]};
\[Omega]StThigh = {0,0,dq[[2]]};
\[Omega]StShank = {0,0,dq[[3]]};
\[Omega]SwThigh = {0,0,dq[[4]]};
\[Omega]SwShank = {0,0,dq[[5]]};

\[Omega] = {\[Omega]Torso, \[Omega]StThigh, \[Omega]StShank, \[Omega]SwThigh, \[Omega]SwShank}; (*Array with all angular velocities*)


(*linear velocities*)
vCOM = D[qCOM, t] //Simplify;  (*gives a matrix for all COM velocities. Each row corresponds to a velocity vector*)
vEnds = D[qEnds, t] //Simplify;  (*gives a matrix of joint and link end velocities. Each row corresponds to a velocity vector*)


(*linear accelerations*)
aEnds = D[vEnds,t] //Simplify;


(* ::Section:: *)
(*Equations of motion*)


(* ::Subsection:: *)
(*Kinetic Energy*)


K = 1/2 Sum[masses[[i]] Dot[vCOM[[i]]//Flatten, vCOM[[i]]//Flatten], {i,1,ndof}]+ 1/2 Sum[Iz[[i]]*Dot[\[Omega][[i]],\[Omega][[i]]], {i,1,ndof}] //ParallelSimplify;


(* ::Subsection:: *)
(*Potential Energy*)


P = Sum[masses[[i]]*g*qCOM[[i,nb]], {i,1,ndof}];


(* ::Subsection:: *)
(*Equations of Motion*)


L = K - P;


eqList = Table[D[D[L,dqe[[i]]],t]-D[L,qe[[i]]],{i,nb+ndof}];


Dimensions[eqList]


Mmat = GetMMatrix[eqList,ddqe]//ParallelSimplify;
Dimensions[Mmat]


Cmat = InertiaToCoriolis[Mmat,qe,dqe];
Dimensions[Cmat]


Gvec = GetGMatrix[eqList,qe];


(* ::Section:: *)
(*Constraints*)


(* constraint Jacobians *)
(* J1, J2, and J3 give the locations of the reaction forces from the exoskeleton *)
J1 = D[{Hip}//Flatten, {qe}];
J2 = D[{StFoot}//Flatten, {qe}];
J3 = D[{SwFoot}//Flatten, {qe}];
(* J4 ensures the torso position at the strap location is the same as that of the exoskeleton *)
  (*J4 = D[{Strap}//Flatten, {qe}] this is redundant if X and Y are included*)
J4 = D[{q[[1]]}//Flatten, {qe}];


J1dot = D[J1,t];
J2dot = D[J2,t];
J3dot = D[J3,t];
J4dot = D[J4,t];


(* ::Section:: *)
(*Output Jacobian*)


(* y(x) is the output vector and is a function of the state vector x = [q, q_dot]' *)
y1 = {qe[[4]],qe[[5]],qe[[6]],qe[[7]]};


(* Jo is the output Jacobian, where y_dot = Jout*x_dot *)
xState = Join[qe,dqe];
Jo1 = D[y1,{xState}];


(* ddy = d(Jy*dx)/dx * dx*)
dxState = Join[dqe,ddqe];
DJo1 = D[Jo1.dxState,{xState}];


(* ::Section:: *)
(*Export to Matlab*)


statesubs=Flatten[Join[Table[{qe[[i]]->x[i],dqe[[i]]->x[nb+ndof+i]},{i,nb+ndof}]]]


generalFormatter[expr_]:=N[expr/.constsubs]/.statesubs;
SetOptions[WriteMatlabFunction,
Formatter->generalFormatter,
Directory->"expr_biped/model",
Arguments->{x}
];


SetDirectory[NotebookDirectory[]]
EnsureDirectoryExists["expr_biped/model"];


WriteMatlabFunction["Mmat_h",Mmat];
WriteMatlabFunction["Cmat_h",Cmat];
WriteMatlabFunction["Gvec_h",Gvec];


WriteMatlabFunction["Jhip_h",J1];
WriteMatlabFunction["JstFoot_h",J2];
WriteMatlabFunction["JswFoot_h",J3];
WriteMatlabFunction["Jtorso_h",J4];
WriteMatlabFunction["Jhipdot_h",J1dot];
WriteMatlabFunction["JstFootdot_h",J2dot];
WriteMatlabFunction["JswFootdot_h",J3dot];
WriteMatlabFunction["Jtorsodot_h",J4dot];


WriteMatlabFunction["qCOM_h",qCOM];
WriteMatlabFunction["qEnds_h",qEnds];
WriteMatlabFunction["vCOM_h",vCOM];
WriteMatlabFunction["vEnds_h",vEnds];
WriteMatlabFunction["aEnds_h",aEnds];


WriteMatlabFunction["Jo1_h",Jo1];
WriteMatlabFunction["DJo1_h",DJo1];
