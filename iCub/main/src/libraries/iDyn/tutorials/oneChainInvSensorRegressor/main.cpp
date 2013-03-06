/** 
* Copyright: 2012
* Author: Silvio Traversaro
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
**/ 

//
// A basic example of a use of iDynRegressor functions to estimate the
// Sensor measurment 
//
//

#include <iostream>
#include <iomanip>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynInv.h>
#include <iCub/iDyn/iDynRegressor.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iDyn;
using namespace iCub::iDyn::Regressor;



// useful print functions
// print a matrix nicely
void printMatrix(string s, const Matrix &m)
{
	cout<<s<<endl;
	for(int i=0;i<m.rows();i++)
	{
		for(int j=0;j<m.cols();j++)
			cout<<setw(15)<<m(i,j);
		cout<<endl;
	}
}
// print a vector nicely
/*
void printVector(string s, const Vector &v)
{
	cout<<s<<endl;
	for(int j=0;j<v.length();j++)
		cout<<setw(15)<<v(j);
	cout<<endl;
}
*/

////////////////
//   MAIN
///////////////

int main()
{
    // In this tutorial, an example of the use of iDynChainRegressorSensorWrench is shown: 
    // given the kinematic and inertial informations  of an arm, the measurment from the FT sensor
    // are estimated, first with iDynInv, and then using iDynChainRegressorSensorWrench
    
	//We define an iCubArmNoTorso with a virtual sensor 
	iCubArmNoTorsoDyn *arm = new iCubArmNoTorsoDyn("right");
    
    iDynInvSensorArmNoTorso *armWSensorSolver = new iDynInvSensorArmNoTorso(arm,DYNAMIC);

    arm->prepareNewtonEuler(DYNAMIC);

  
    Vector q(arm->getN()); q.zero();
    Vector dq(q); Vector ddq(q);
    
    //We assign some arbitrary values
    for(int i=0; i < arm->getN(); i++ ) {
        q[i] = i*10.0;
        dq[i] = i*5.0;
        ddq[i] = i*3.0;
    }
    
    
    // now we set the joints values
    // note: if the joint angles exceed the joint limits, their value is saturated to the min/max automatically
    // note: STATIC computations only use q, while DYNAMIC computations use q,dq,ddq.. but we set everything
    q = arm->setAng(q);
    dq = arm->setDAng(dq);
    ddq = arm->setD2Ang(ddq);

    // the inertial information to initialize the kinematic phase
    // this information must be set at the base of the chain, where the base is intended to be the first link (first.. in the Denavit
    // Hartenberg convention)
    // note: if nothing is moving, it can be simply zero, but ddp0 must provide the gravity information :)
	Vector w0(3); Vector dw0(3); Vector ddp0(3);
	w0=dw0=ddp0=0.0; ddp0[2]=9.81;
    Vector Fend(3); Vector Mend(3);
    Fend = Mend = 0.0;
    arm->computeNewtonEuler(w0,dw0,ddp0,Fend,Mend);
    
    
    armWSensorSolver->computeSensorForceMoment();
    
    //Finally, the Sensor wrench is calculated
    Vector W_with_idyn = armWSensorSolver->getSensorForceMoment();
    
    //For doing the same thing using the Regressor, first the dynamical
    //parameters information must be extracted from the iDynChain and the iDynInvSensor
    //objects, as the subchain of the link after the sensor is going to be considered
    //note: in this example, we will consider all the links except for the 5th, because it is
    //a virtual link
    Vector beta;
    const int virtual_link = 5;
    iDynChainGetBeta(arm->asChain(),(iDynSensor*)armWSensorSolver,beta,virtual_link);
    
    //Then the regressor is calculated 
    Matrix Phi;
    iDynChainRegressorSensorWrench(arm->asChain(),(iDynSensor*)armWSensorSolver,Phi,virtual_link);
    
    //And using this two information, the Wrench measured from the sensor can be calculated;
    Vector W_with_regr = Phi*beta;
    
    // now we can print the information, and see that the wrench is equal
    cout<<"iCubArmNoTorsoDyn has "<<arm->getN()<<" links."<<endl;

    cout<<"\n\n";

    printVector("Estimated Wrench in the FT sensor calculated by iDynInv",W_with_idyn);

    cout<<"\n\n";

    printVector("Estimated Wrench in the FT sensor calculated by iDynRegressor",W_with_regr);

    cout << "\n\n";

    //exit
	cin.get();

    // remember to delete everything before exiting :)
    cout<<"\nDestroying sensor..."<<endl;
    delete armWSensorSolver; armWSensorSolver=NULL;
    cout<<"Destroying arm..."<<endl;
    delete arm; arm=NULL;

    return 0;
}
      
      
