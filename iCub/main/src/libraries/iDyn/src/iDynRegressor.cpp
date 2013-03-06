#include <iCub/iDyn/iDynRegressor.h>

#include <yarp/os/Log.h>
#include <iCub/ctrl/math.h>

#include <iostream>


//should be fixed in iDyn 
#define CAST_IKINLINK_TO_IDYNLINK

using namespace std;

using namespace yarp::sig;
using namespace yarp::math;

using namespace iCub::iDyn;
using namespace iCub::ctrl;

using namespace iCub::iDyn::Regressor;


bool iCub::iDyn::Regressor::iCubLimbRegressorSensorWrench(iCubWholeBody * icub, const std::string & limbName,Matrix & Phi, bool consider_virtual_link)
{
    iDynChain * p_chain;
    iDynSensor * p_sensor;
    int VIRTUAL_LINK;
	if( iCubLimbGetData(icub,limbName,consider_virtual_link,p_chain,p_sensor,VIRTUAL_LINK) == false ) return false;
	iDynChainRegressorSensorWrench(p_chain,p_sensor,Phi,VIRTUAL_LINK);
    return true;
}

bool iCub::iDyn::Regressor::iCubLimbRegressorComplete(iCubWholeBody * icub, const std::string & limbName,Matrix & Phi, bool consider_virtual_link)
{
   iDynChain * p_chain;
    iDynSensor * p_sensor;
	int VIRTUAL_LINK;
	if( iCubLimbGetData(icub,limbName,consider_virtual_link,p_chain,p_sensor,VIRTUAL_LINK) == false ) return false;
	 YARP_ASSERT(iDynChainRegressorComplete(p_chain,p_sensor,Phi,VIRTUAL_LINK));
     return true;
}

bool iCub::iDyn::Regressor::iCubLimbRegressorInternalWrench(iCubWholeBody * icub, const std::string & limbName,int wrench_index,Matrix & Phi, bool consider_virtual_link)
{
	iDynChain * p_chain;
    iDynSensor * p_sensor;
    int VIRTUAL_LINK;
    if( iCubLimbGetData(icub,limbName,consider_virtual_link,p_chain,p_sensor,VIRTUAL_LINK) == false ) return false;
	iDynChainRegressorInternalWrench(p_chain,p_sensor,Phi,wrench_index,VIRTUAL_LINK);
    return true;
}

bool iCub::iDyn::Regressor::iDynChainRegressorSensorWrench(iDynChain * p_chain,iDynSensor * p_sensor, Matrix & A,const int excluded_link)
{
    vector<bool> excluded_links;
    excluded_links.resize(p_chain->getN());
	if( setOnlyOneElement(excluded_links,excluded_link) == false ) return false;
    return iDynChainRegressorSensorWrench(p_chain,p_sensor,A,excluded_links);
}

bool iCub::iDyn::Regressor::iDynChainRegressorSensorWrench(iDynChain *  p_chain, iDynSensor * p_sensor, Matrix & A, const vector<bool> excluded_links)
{
    const int SENSOR_LINK_INDEX = p_sensor->getSensorLink();
    const int FINAL_LINK_INDEX = p_chain->getN()-1;
    int VIRTUAL_LINKS = 0;
    for(int i=SENSOR_LINK_INDEX;i <= FINAL_LINK_INDEX;i++) { if(excluded_links[i]) { VIRTUAL_LINKS++; } }
    const int TOTAL_IDENT_LINKS = FINAL_LINK_INDEX - SENSOR_LINK_INDEX+1-VIRTUAL_LINKS;
    A = Matrix(6,10*(TOTAL_IDENT_LINKS));
    A.zero();
    
    Matrix H_current = Matrix(4,4);
    Matrix G = Matrix(3,3);
    int j = 0;
    for(int link_index = SENSOR_LINK_INDEX;link_index <= FINAL_LINK_INDEX; link_index++) {
        iCub::iKin::iKinLink & link_current = (*p_chain)[link_index];
        if( link_index == SENSOR_LINK_INDEX ) {
            //the H contained in the sensor is \f$ H^s_i
            H_current = SE3inv(p_sensor->getH());
        } else {
            H_current = H_current * link_current.getH();
        }
        if( !excluded_links[link_index] ) { 
            iDynLink * p_link = (iDynLink *) &link_current;
            const int start_col = 10*j;
            j++;
            A.setSubmatrix(adjointInv(H_current).transposed()*iDynLinkRegressorNetWrench(p_link),0,start_col);
        }
    }
    return true;
}

bool iCub::iDyn::Regressor::iDynChainRegressorComplete(iDynChain * p_chain,iDynSensor * p_sensor, Matrix & A,const int excluded_link)
{
    vector<bool> excluded_links;
    excluded_links.resize(p_chain->getN());
	if( setOnlyOneElement(excluded_links,excluded_link) == false ) return false;
    return iDynChainRegressorComplete(p_chain,p_sensor,A,excluded_links);
}

bool iCub::iDyn::Regressor::iDynChainRegressorComplete(iDynChain *  p_chain, iDynSensor * p_sensor, Matrix & A, const vector<bool> excluded_links)
{
    const int SENSOR_LINK_INDEX = p_sensor->getSensorLink();
    const int FINAL_LINK_INDEX = p_chain->getN()-1;
    int VIRTUAL_LINKS = 0;
    for(int i=SENSOR_LINK_INDEX;i <= FINAL_LINK_INDEX;i++) { if(excluded_links[i]) { VIRTUAL_LINKS++; } }
    const int TOTAL_IDENT_LINKS = FINAL_LINK_INDEX - SENSOR_LINK_INDEX+1-VIRTUAL_LINKS;
    //cout << "Final link index " << FINAL_LINK_INDEX << " SENSOR_LINK_INDEX " << SENSOR_LINK_INDEX << endl;
    A = Matrix(6+FINAL_LINK_INDEX-SENSOR_LINK_INDEX,10*(TOTAL_IDENT_LINKS));
    A.zero();
    
    Matrix H_current = Matrix(4,4);
    Matrix G = Matrix(3,3);
    int j = 0;
    for(int link_index = SENSOR_LINK_INDEX;link_index <= FINAL_LINK_INDEX; link_index++) {
        iCub::iKin::iKinLink & link_current = (*p_chain)[link_index];
        if( link_index == SENSOR_LINK_INDEX ) {
            //the sensor H is not the right one
            H_current = SE3inv(p_sensor->getH());
        } else {
            H_current = H_current * link_current.getH();
        }
        //Just for debug
        //H_current = H_wrt_sensor(p_chain,p_sensor,link_index);
        if( !excluded_links[link_index] ) { 
            iDynLink * p_link = (iDynLink *) &link_current;
            const int start_col = 10*j;
            j++;
            A.setSubmatrix(adjointInv(H_current).transposed()*iDynLinkRegressorNetWrench(p_link),0,start_col);
            //Setting lower rows
            for(int p=SENSOR_LINK_INDEX+1; p <= FINAL_LINK_INDEX; p++ ) {
				int start_row = 6+p-SENSOR_LINK_INDEX-1;
				if( p <= link_index ) {
					A.setSubrow((adjointInv(H_b_wrt_a(p_chain,p-1,link_index)).transposed()*iDynLinkRegressorNetWrench(p_link)).getRow(5),start_row,start_col);
				} else {
					Vector allZero(A.cols());
					allZero.zero();
					A.setSubrow(allZero,start_row,start_col);
				}
			}
        }
    }
    return true;
}


bool iCub::iDyn::Regressor::iDynChainRegressorInternalWrench(iDynChain * p_chain,iDynSensor * p_sensor, Matrix & A,int wrench_index,const int excluded_link)
{
    vector<bool> excluded_links;
    excluded_links.resize(p_chain->getN());
	if( setOnlyOneElement(excluded_links,excluded_link) == false ) return false;
    return iDynChainRegressorInternalWrench(p_chain,p_sensor,A,wrench_index,excluded_links);
}

bool iCub::iDyn::Regressor::iDynChainRegressorInternalWrench(iDynChain *  p_chain, iDynSensor * p_sensor, Matrix & A,int wrench_index, const vector<bool> excluded_links)
{
    YARP_ASSERT(wrench_index >= (int) p_sensor->getSensorLink());
    YARP_ASSERT(wrench_index < (int) (p_chain->getN()) );
    const int SENSOR_LINK_INDEX = p_sensor->getSensorLink();
    const int FINAL_LINK_INDEX = p_chain->getN()-1;
    int VIRTUAL_LINKS = 0;
    for(int i=SENSOR_LINK_INDEX;i <= FINAL_LINK_INDEX;i++) { if(excluded_links[i]) { VIRTUAL_LINKS++; } }
    const int TOTAL_IDENT_LINKS = FINAL_LINK_INDEX - SENSOR_LINK_INDEX+1-VIRTUAL_LINKS;
    A = Matrix(6,10*(TOTAL_IDENT_LINKS));
    A.zero();
    Matrix H_current;
    
    int j = 0;
    for(int link_index = SENSOR_LINK_INDEX;link_index <= FINAL_LINK_INDEX; link_index++) {
        iCub::iKin::iKinLink & link_current = (*p_chain)[link_index];
        //Just for debug
        H_current = H_b_wrt_a(p_chain,wrench_index,link_index);
        if( !excluded_links[link_index] ) { 
            iDynLink * p_link = (iDynLink *) &link_current;
            const int start_col = 10*j;
            j++;
            if( wrench_index < link_index ) {
				A.setSubmatrix(adjointInv(H_current).transposed()*iDynLinkRegressorNetWrench(p_link),0,start_col);
			} 
        }
    }
    return true;
}


Vector iCub::iDyn::Regressor::iDynChainRegressorTorqueEstimation(iDynChain * p_chain,iDynSensor * p_sensor,const int joint_index, const int excluded_link)
{
    vector<bool> excluded_links;
    excluded_links.resize(p_chain->getN());
	//if( setOnlyOneElement(excluded_links,excluded_link) == false ); //return false;//TODO
    setOnlyOneElement(excluded_links,excluded_link);
    return iDynChainRegressorTorqueEstimation(p_chain,p_sensor,joint_index,excluded_links);
}

Vector iCub::iDyn::Regressor::iDynChainRegressorTorqueEstimation(iDynChain *p_chain,iDynSensor * p_sensor, const int joint_index, vector<bool> excluded_links)
{
    YARP_ASSERT(joint_index > (int) p_sensor->getSensorLink());
    YARP_ASSERT(joint_index < (int) (p_chain->getN()) );
    
    const int SENSOR_LINK_INDEX = p_sensor->getSensorLink();
    const int FINAL_LINK_INDEX = p_chain->getN()-1;
    int VIRTUAL_LINKS = 0;
    for(int i=SENSOR_LINK_INDEX;i <= FINAL_LINK_INDEX;i++) { if(excluded_links[i]) { VIRTUAL_LINKS++; } }
    const int TOTAL_IDENT_LINKS = FINAL_LINK_INDEX - SENSOR_LINK_INDEX+1-VIRTUAL_LINKS;
    Matrix A = Matrix(6,10*(TOTAL_IDENT_LINKS));
    A.zero();
    Matrix H_current;
    int j = 0;
    int joint_link_index = joint_index-1;
    for(int link_index = SENSOR_LINK_INDEX;link_index <= joint_link_index; link_index++) {
        iCub::iKin::iKinLink & link_current = (*p_chain)[link_index];
        H_current = H_b_wrt_a(p_chain,joint_link_index,link_index);
    
        if( !excluded_links[link_index] ) { 
            iDynLink * p_link = (iDynLink *) &link_current;
            const int start_col = 10*j;
            j++;
            A.setSubmatrix(-1*adjointInv(H_current).transposed()*iDynLinkRegressorNetWrench(p_link),0,start_col);
        }
    }
    return A.getRow(5);
}

Matrix iCub::iDyn::Regressor::iDynChainRegressorWrenchEstimation(iDynChain * p_chain,iDynSensor * p_sensor,const int link_index, const int excluded_link)
{
    vector<bool> excluded_links;
	//if( setOnlyOneElement(excluded_links,excluded_link) == false ) return false;
	//TODO error check
	setOnlyOneElement(excluded_links,excluded_link);
    return iDynChainRegressorWrenchEstimation(p_chain,p_sensor,link_index,excluded_links);
}

Matrix iCub::iDyn::Regressor::iDynChainRegressorWrenchEstimation(iDynChain *p_chain,iDynSensor * p_sensor, const int link_index, vector<bool> excluded_links)
{
    YARP_ASSERT(link_index >= (int) p_sensor->getSensorLink());
    YARP_ASSERT(link_index < (int) (p_chain->getN()) );
    const int SENSOR_LINK_INDEX = p_sensor->getSensorLink();
    const int FINAL_LINK_INDEX = p_chain->getN()-1;
    int VIRTUAL_LINKS = 0;
    for(int i=SENSOR_LINK_INDEX;i <= FINAL_LINK_INDEX;i++) { if(excluded_links[i]) { VIRTUAL_LINKS++; } }
    const int TOTAL_IDENT_LINKS = FINAL_LINK_INDEX - SENSOR_LINK_INDEX+1-VIRTUAL_LINKS;
    Matrix A = Matrix(6,10*(TOTAL_IDENT_LINKS));
    A.zero();
    Matrix H_current;
    int j = 0;
    for(int current_link_index = SENSOR_LINK_INDEX;current_link_index <= link_index; current_link_index++) {
        iCub::iKin::iKinLink & link_current = (*p_chain)[current_link_index];
        H_current = H_b_wrt_a(p_chain,link_index,current_link_index);
    
        if( !excluded_links[current_link_index] ) { 
            iDynLink * p_link = (iDynLink *) &link_current;
            const int start_col = 10*j;
            j++;
            A.setSubmatrix(-1*adjointInv(H_current).transposed()*iDynLinkRegressorNetWrench(p_link),0,start_col);
        }
    }
    return A;
}

Matrix iCub::iDyn::Regressor::iDynLinkRegressorNetWrench(iDynLink * p_link)
{
    Matrix A(6,10);
    A.zero();
    Vector ddp = p_link->getLinAcc();
    Vector w = p_link->getW();
    Vector dw = p_link->getdW();
    A.setSubcol(ddp,0,0);
    /*
    Vector vector_zero = Vector(3);
    vector_zero.zero();
    A.setSubcol(vector_zero,3,0);
    */
    A.setSubmatrix(crossProductMatrix(dw)+crossProductMatrix(w)*crossProductMatrix(w),0,1);
    A.setSubmatrix(-1*crossProductMatrix(ddp),3,1);
    /*
    Matrix mat_zero = Matrix(3,6);
    mat_zero.zero();
    A.setSubmatrix(mat_zero,0,1+3);
    */
    A.setSubmatrix(EulerEquationsRegressor(w,dw),3,1+3);
    return A;
}

Matrix iCub::iDyn::Regressor::EulerEquationsRegressor(const Vector & w,const Vector& dw)
{
    YARP_ASSERT(w.size() == 3);
    YARP_ASSERT(dw.size() == 3);
    Matrix ret = Matrix(3,6);
    ret.zero();
    //first the pure mixed product of elements of w, as this calculation is then used
    ret(0,5) = w[1]*w[2];
    ret(1,0) = w[0]*w[2];
    ret(2,3) = w[0]*w[1];
    
    ret(0,0) = dw[0];
    ret(0,1) = dw[1] - ret(1,0);
    ret(0,2) = dw[2] + ret(2,3);
    ret(0,3) = -ret(0,5);
    ret(0,4) = w[1]*w[1] - w[2]*w[2];
    //ret(0,5) = w[1]*w[2];
    //ret(1,0) = w[0]*w[2];
    ret(1,1) = dw[0] + ret(0,5);
    ret(1,2) = w[2]*w[2] - w[0]*w[0];
    ret(1,3) = dw[1];
    ret(1,4) = dw[2] - ret(2,3);
    ret(1,5) = -ret(1,0);
    ret(2,0) = -ret(2,3); 
    ret(2,1) = w[0]*w[0] - w[1]*w[1];
    ret(2,2) = dw[0] - ret(0,5);
    //ret(2,3) = w[0]*w[1];
    ret(2,4) = dw[1] + ret(1,0);
    ret(2,5) = dw[2];
    return ret;
}

Matrix iCub::iDyn::Regressor::H_wrt_sensor(iDynChain * p_chain, iDynSensor * p_sensor, const unsigned int index, bool inv)
{
    YARP_ASSERT(index >= p_sensor->getSensorLink());
    YARP_ASSERT(index < p_chain->getN());
    const int SENSOR_LINK_INDEX = p_sensor->getSensorLink();
    Matrix H_current = Matrix(4,4);
    for(int link_index = SENSOR_LINK_INDEX;link_index <= (int)index; link_index++) {
        iCub::iKin::iKinLink & link_current = (*p_chain)[link_index];
        if( link_index == SENSOR_LINK_INDEX ) {
            //the sensor H is not the right one
            H_current = SE3inv(p_sensor->getH());
        } else {
            H_current = H_current * link_current.getH();
        }
    }
    if(!inv) { 
        return H_current;
    } else {
        return SE3inv(H_current);
    }
}

Matrix iCub::iDyn::Regressor::H_b_wrt_a(iDynChain * p_chain, int a_index, int b_index)
{
    YARP_ASSERT(a_index < (int) p_chain->getN() );
    YARP_ASSERT(b_index < (int) p_chain->getN() );
    Matrix H(4,4);
    if( a_index == b_index ) {
        H.eye();
        return H;
    }
    bool return_inv = false;
    if(a_index > b_index ) {
        int swap;
        return_inv = true;
        swap = a_index;
        a_index = b_index;
        b_index = swap;
    }
    for(int i = a_index+1; i <= b_index; i++) {
        iCub::iKin::iKinLink & link_current = (*p_chain)[i];
        if(i== a_index+1 ) {
            H = link_current.getH();
        } else {
            H = H * link_current.getH();
        }
    }
    if( !return_inv) {
        return H;
    } else {
        return SE3inv(H);
    }
}

bool iCub::iDyn::Regressor::setOnlyOneElement(std::vector<bool> & bool_vec, const int indexToSet)
{
	if( indexToSet >= (int) bool_vec.size() ) return false;
	for(int i=0; i < (int)bool_vec.size(); i++ ) {
        if( indexToSet == i ) {
            bool_vec[i] = true;
        } else {
            bool_vec[i] = false;
        }
    }
    return true;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

bool iCub::iDyn::Regressor::iCubLimbGetBeta(const iCubWholeBody * const icub,std::string limbName,Vector & beta, bool consider_virtual_link)
{
    iDynChain * p_chain;
    iDynSensor * p_sensor;
    int VIRTUAL_LINK;
	if( iCubLimbGetData(icub,limbName,consider_virtual_link,p_chain,p_sensor,VIRTUAL_LINK) == false ) return false;

    if( iDynChainGetBeta(p_chain,p_sensor,beta,VIRTUAL_LINK) == false ) return false;
    return true;
}

bool iCub::iDyn::Regressor::iDynChainGetBeta(iDynChain * p_chain,iDynSensor * p_sensor, Vector & beta,const int excluded_link)
{
    vector<bool> excluded_links;
    excluded_links.resize(p_chain->getN());
	if( setOnlyOneElement(excluded_links,excluded_link) == false ) return false;
    return iDynChainGetBeta(p_chain,p_sensor,beta,excluded_links);
}

bool iCub::iDyn::Regressor::iDynChainGetBeta(iDynChain * p_chain, iDynSensor * p_sensor, Vector & beta,const vector<bool> excluded_links)
{
    Vector beta_i = Vector(10);
    Vector beta_ident = Vector(10);
    const int SENSOR_LINK_INDEX = p_sensor->getSensorLink();
    const int FINAL_LINK_INDEX = p_chain->getN()-1;
    int VIRTUAL_LINKS = 0;
    for(int i=SENSOR_LINK_INDEX;i <= FINAL_LINK_INDEX;i++) { if(excluded_links[i]) { VIRTUAL_LINKS++; } }
    const int TOTAL_IDENT_LINKS = FINAL_LINK_INDEX - SENSOR_LINK_INDEX+1-VIRTUAL_LINKS;
    beta = Vector(10*TOTAL_IDENT_LINKS);
    beta.zero();
        int j=0;
        for(int i=SENSOR_LINK_INDEX;i<=FINAL_LINK_INDEX;i++) {
            //fprintf(stderr,"i = %d\n",i); 
            if( i == SENSOR_LINK_INDEX) {
                Vector beta_semiLink(10);
                beta_semiLink[0]= p_sensor->getMass();
                //attention, SensorLinkDoesNotstore the usual r_{i,C}, but r_{s,C}
                beta_semiLink.setSubvector(1,(p_sensor->getCOM()).subcol(0,3,3));
                beta_semiLink.setSubvector(1+3,symmMatrix2Vector(p_sensor->getInertia()));
                param_conv_semiLink2idyn(beta_semiLink,p_sensor->getH(),beta_i);
                
            } else {
                if( !excluded_links[i] ) {
                //p_link = p_chain->refLink(i);
                    beta_i[0]= p_chain->getMass(i);
                    beta_i.setSubvector(1,(p_chain->getCOM(i)).subcol(0,3,3));
                    beta_i.setSubvector(1+3,symmMatrix2Vector(p_chain->getInertia(i)));
                }
            }
            if( !excluded_links[i] ) {
                param_conv_idyn2ident(beta_i,beta_ident);
                beta.setSubvector((j)*10,beta_ident);
                j++;
            }
        }
    return true;
}


bool iCub::iDyn::Regressor::iDynChainGetLinkBeta(iDynChain * p_chain,const unsigned link,Vector & beta_ident)
{
	Vector beta_i(10);
	beta_i[0]= p_chain->getMass(link);
    beta_i.setSubvector(1,(p_chain->getCOM(link)).subcol(0,3,3));
    beta_i.setSubvector(1+3,symmMatrix2Vector(p_chain->getInertia(link)));
    param_conv_idyn2ident(beta_i,beta_ident);
    return true;
}

bool iCub::iDyn::Regressor::iDynSensorGetSemilinkBeta(const iDynSensor *p_sensor,Vector & beta_ident)
{
	Vector beta_semiLink(10),beta_idyn(10);
	beta_semiLink[0]= p_sensor->getMass();
    //attention, SensorLink does not store the usual r_{i,C}, but r_{s,C}
    beta_semiLink.setSubvector(1,(p_sensor->getCOM()).subcol(0,3,3));
    beta_semiLink.setSubvector(1+3,symmMatrix2Vector(p_sensor->getInertia()));
    param_conv_semiLink2idyn(beta_semiLink,p_sensor->getH(),beta_idyn);
    param_conv_idyn2ident(beta_idyn,beta_ident);
    return true;
}




void iCub::iDyn::Regressor::param_conv_idyn2ident(const Vector& beta_idyn, Vector & beta_ident)
{
    YARP_ASSERT(beta_idyn.size() == 10);
    YARP_ASSERT(beta_ident.size() == 10);
    beta_ident[0] = beta_idyn[0];
    beta_ident[1] = beta_idyn[0]*beta_idyn[1];
    beta_ident[2] = beta_idyn[0]*beta_idyn[2];
    beta_ident[3] = beta_idyn[0]*beta_idyn[3];
    beta_ident[4] = beta_idyn[4] + beta_idyn[0]*(beta_idyn[2]*beta_idyn[2]+beta_idyn[3]*beta_idyn[3]);
    beta_ident[5] = beta_idyn[5] - beta_idyn[0]*beta_idyn[1]*beta_idyn[2];
    beta_ident[6] = beta_idyn[6] - beta_idyn[0]*beta_idyn[1]*beta_idyn[3];
    beta_ident[7] = beta_idyn[7] + beta_idyn[0]*(beta_idyn[1]*beta_idyn[1]+beta_idyn[3]*beta_idyn[3]);
    beta_ident[8] = beta_idyn[8] - beta_idyn[0]*beta_idyn[2]*beta_idyn[3];
    beta_ident[9] = beta_idyn[9] + beta_idyn[0]*(beta_idyn[1]*beta_idyn[1]+beta_idyn[2]*beta_idyn[2]);
}

void iCub::iDyn::Regressor::param_conv_ident2idyn(const Vector  &beta_ident,Vector & beta_idyn)
{
    YARP_ASSERT(beta_ident.size() == 10);
    YARP_ASSERT(beta_idyn.size() == 10);
    if( beta_ident[0] != 0 ) {
    beta_idyn = Vector(10);
    beta_idyn[0] = beta_ident[0];
    beta_idyn[1] = beta_ident[1]/beta_ident[0];
    beta_idyn[2] = beta_ident[2]/beta_ident[0];
    beta_idyn[3] = beta_ident[3]/beta_ident[0];
    beta_idyn[4] = beta_ident[4] - (beta_ident[2]*beta_ident[2]+beta_ident[3]*beta_ident[3])/beta_ident[0];
    beta_idyn[5] = beta_ident[5] + (beta_ident[1]*beta_ident[2])/beta_ident[0];
    beta_idyn[6] = beta_ident[6] + (beta_ident[1]*beta_ident[3])/beta_ident[0];
    beta_idyn[7] = beta_ident[7] - (beta_ident[1]*beta_ident[1]+beta_ident[3]*beta_ident[3])/beta_ident[0];
    beta_idyn[8] = beta_ident[8] + (beta_ident[2]*beta_ident[3])/beta_ident[0];
    beta_idyn[9] = beta_ident[9] - (beta_ident[1]*beta_ident[1]+beta_ident[2]*beta_ident[2])/beta_ident[0];
    } else {
        //if the mass of a link is zero, all other parameters are 0
        beta_idyn.zero();
    }
}


void iCub::iDyn::Regressor::param_conv_semiLink2idyn(const Vector  &beta_semiLink, const Matrix & H_s_i, Vector & beta_idyn)
{
    Vector r_sC_s(3),r_iC_i(3);
    Matrix I_s(3,3),I_i(3,3);
    Matrix R_s_i = H_s_i.submatrix(0,2,0,2);
    Vector r_is_i = H_s_i.subcol(0,3,3);
    
    //The mass is the same, as it is a scalar value
    beta_idyn[0] = beta_semiLink[0];
    
    //The center of mass must be referred to the i frame
    //r_{i,C}^i = r_{i,s}^i + R_s^i r_{s,C}^s
    r_sC_s = beta_semiLink.subVector(1,3);
    r_iC_i = r_is_i + R_s_i*r_sC_s;
    beta_idyn.setSubvector(1,r_iC_i);
    
    //The inertia matrix must be referred to the i frame
    I_s = Vector2symmMatrix(beta_semiLink.subVector(4,9));
    I_i = R_s_i*I_s*(R_s_i.transposed());
    beta_idyn.setSubvector(4,symmMatrix2Vector(I_i));
    
    return;
}

void iCub::iDyn::Regressor::param_conv_idyn2semiLink(const Vector  &beta_idyn,const Matrix & H_s_i,Vector & beta_semiLink)
{
    Vector r_sC_s(3),r_iC_i(3);
    Matrix I_s(3,3),I_i(3,3);
    Matrix R_s_i = H_s_i.submatrix(0,2,0,2);
    Vector r_is_i = H_s_i.subcol(0,3,3);
    
    //The mass is the same, as it is a scalar value
    beta_semiLink[0] = beta_idyn[0];
    
    //The center of mass must be referred to the i frame
    //r_{s,C}^s = R_i^s (r_{i,C}^i - r_{i,s}^i)
    r_iC_i = beta_idyn.subVector(1,3);
    r_sC_s = R_s_i.transposed()*(r_iC_i - r_is_i);
    beta_semiLink.setSubvector(1,r_sC_s);
    
    //The inertia matrix must be referred to the i frame
    I_i = Vector2symmMatrix(beta_idyn.subVector(4,9));
    //printMatrix("I_i",I_i);
    I_s = (R_s_i.transposed())*I_i*(R_s_i);
    //printMatrix("I_s",I_s);
    beta_semiLink.setSubvector(4,symmMatrix2Vector(I_s));
    
    return;
}




double iCub::iDyn::Regressor::param_conv_idyn2mass(const Vector & beta_idyn)
{
    return beta_idyn[0];
}

Vector iCub::iDyn::Regressor::param_conv_idyn2com(const Vector & beta_idyn)
{
    return beta_idyn.subVector(1,3);
}

Matrix iCub::iDyn::Regressor::param_conv_idyn2COM(const Vector & beta_idyn)
{
    Matrix ret = Matrix(4,4);
    ret.zero();
    ret.setSubcol(beta_idyn.subVector(1,3),0,3);
    ret.setSubmatrix(eye(3),0,0);
    return ret;
}


Matrix iCub::iDyn::Regressor::param_conv_idyn2inertia(const Vector & beta_idyn)
{
    return Vector2symmMatrix(beta_idyn.subVector(4,9));
}

Vector iCub::iDyn::Regressor::symmMatrix2Vector(const Matrix& symMat)
{
    YARP_ASSERT(symMat(0,1) == symMat(1,0));
    YARP_ASSERT(symMat(0,2) == symMat(2,0));
    YARP_ASSERT(symMat(1,2) == symMat(2,1));
    Vector ret = Vector(6);
    ret[0] = symMat(0,0);
    ret[1] = symMat(0,1);
    ret[2] = symMat(0,2);
    ret[3] = symMat(1,1);
    ret[4] = symMat(1,2);
    ret[5] = symMat(2,2);
    return ret;
}

Matrix iCub::iDyn::Regressor::Vector2symmMatrix(const Vector & ret)
{
    Matrix symMat = Matrix(3,3);
    symMat.zero();
    symMat(0,0) = ret[0];
    symMat(0,1) = symMat(1,0) = ret[1];
    symMat(0,2) = symMat(2,0) = ret[2];
    symMat(1,1) = ret[3];
    symMat(1,2) = symMat(2,1) = ret[4];
    symMat(2,2) = ret[5];
    return symMat;
}

bool iCub::iDyn::Regressor::iDynChainSetBeta(iDynChain * p_chain, iDynSensor * p_sensor, const Vector & beta,int excluded_link)
{
    vector<bool> excluded_links;
    excluded_links.resize(p_chain->getN());
	if( setOnlyOneElement(excluded_links,excluded_link) == false ) return false;
    return iDynChainSetBeta(p_chain,p_sensor,beta,excluded_links);
}

bool iCub::iDyn::Regressor::iDynChainSetBeta(iDynChain * p_chain, iDynSensor * p_sensor, const Vector & beta,vector<bool> excluded_links)
{
	const int SENSOR_LINK_INDEX = p_sensor->getSensorLink();
    const int FINAL_LINK_INDEX = p_chain->getN()-1;
    Vector beta_ident(10),beta_idyn(10),beta_semiLink(10),beta_ident_sensor_link(10),beta_ident_old(10),beta_ident_sensor_link_old(10), beta_idyn_sensor_link(10);
    int l = 0;
    //It is necessary to update the dynamical parameters of the link containing the sensor
    //beta_ident new dynamical parameters of the semilink
    //beta_ident_sensor_link new dynamical parameters of the sensor link
    //beta_ident_old old dynamical parameters of the semilink
    //beta_ident_sensor_link_old old dynamical parameters of the sensor link
    beta_ident = beta.subVector(0,9);
	l++;
    iDynSensorGetSemilinkBeta(p_sensor,beta_ident_old);
    iDynChainGetLinkBeta(p_chain,SENSOR_LINK_INDEX,beta_ident_sensor_link_old);
    
    beta_ident_sensor_link = beta_ident_sensor_link_old + (beta_ident - beta_ident_old);
    param_conv_ident2idyn(beta_ident_sensor_link,beta_idyn_sensor_link);
    p_chain->setDynamicParameters(SENSOR_LINK_INDEX,param_conv_idyn2mass(beta_idyn_sensor_link),param_conv_idyn2COM(beta_idyn_sensor_link),param_conv_idyn2inertia(beta_idyn_sensor_link));

     
    param_conv_ident2idyn(beta_ident,beta_idyn);
    param_conv_idyn2semiLink(beta_idyn,p_sensor->getH(),beta_semiLink);
    p_sensor->setDynamicParameters(param_conv_idyn2mass(beta_semiLink),param_conv_idyn2COM(beta_semiLink),param_conv_idyn2inertia(beta_semiLink)); 
    
    //then assign the remaining links
    for(int link = SENSOR_LINK_INDEX+1;link <= FINAL_LINK_INDEX; link++ ) {
        if( !excluded_links[link] ) {
            beta_ident = beta.subVector(l*10,l*10+9);
            l++;
            param_conv_ident2idyn(beta_ident,beta_idyn);
            p_chain->setDynamicParameters(link,param_conv_idyn2mass(beta_idyn),param_conv_idyn2COM(beta_idyn),param_conv_idyn2inertia(beta_idyn));
        }
    }
    return true;
} 


bool iCub::iDyn::Regressor::iCubLimbSetBeta(iCubWholeBody * icub,std::string limbName,const Vector & beta,bool consider_virtual_link)
{
    iDynChain * p_chain;
    iDynSensor * p_sensor;
	int virtual_link;
	YARP_ASSERT(beta.size() > 0)
	if( iCubLimbGetData(icub,limbName,consider_virtual_link,p_chain,p_sensor,virtual_link) == false ) return false;
    return iDynChainSetBeta(p_chain,p_sensor,beta,virtual_link);
}

//shoul be change to GetMetadata
bool iCub::iDyn::Regressor::iCubLimbGetData(const iCubWholeBody * const icub, const std::string & limbName,const bool consider_virtual_link, iDynChain * & p_chain, iDynSensor * & p_sensor,int & virtual_link )
{
	if( limbName == "right_arm" || limbName == "left_arm" || limbName == "left_leg" || limbName == "right_leg") {
        if( consider_virtual_link ) {
			virtual_link = -1;
		} else {
			if( limbName == "right_arm" || limbName == "left_arm" ) {
				virtual_link = 5; 
			} 
			if( limbName == "left_leg" || limbName == "right_leg" ) {
				virtual_link = 5; //By chance!
			}
		}
		if( limbName == "right_arm" ) {
			p_chain =  icub->upperTorso->right->asChain();
			p_sensor = icub->upperTorso->rightSensor;
		}
		if( limbName== "left_arm" ) {
			p_chain =  icub->upperTorso->left->asChain();
			p_sensor = icub->upperTorso->leftSensor;
		}
		if( limbName == "right_leg" ) {
			p_chain =  icub->lowerTorso->right->asChain();
			p_sensor = icub->lowerTorso->rightSensor;
		}
		if( limbName== "left_arm" ) {
			p_chain =  icub->lowerTorso->left->asChain();
			p_sensor = icub->lowerTorso->leftSensor;
		}
		return true;
	} else {
		fprintf(stderr,"limb name non recognized\n");
		return false;
	}
}

/*
bool iDynChainExtractSubChain(iDynChain & input_chain, iDynSensor & input_sensor, iDynChain & sub_chain)
{
    const int sensor_link_index = input_sensor.getSensorLink();
    const int final_link_index = input_chain.getN()-1;
    sub_chain = iDynChain();
    
    Matrix H0(4,4);
    //Probably it is not necessary, however
    input_chain.computeKinematicNewtonEuler();
    
    //Putting all the rotation and displacement in H0, and 
    //the DH parameters of the semilink to 0,0,0,0 (otherwise an arbitrary
    // DH compatible frame of reference should be found, and no sensed one 
    // appears to me now)
    //H0 = H^s_i 
	H0 =  SE3inv(input_sensor.getH());
	sub_chain.setH0(H0);
    
    Vector beta_semiLink(10), beta_idyn(10);
    beta_semiLink[0]= input_sensor.getMass();
    //attention, SensorLinkDoesNotstore the usual r_{i,C}, but r_{s,C}
    beta_semiLink.setSubvector(1,(input_sensor.getCOM()).subcol(0,3,3));
    beta_semiLink.setSubvector(1+3,symmMatrix2Vector(input_sensor.getInertia()));
    param_conv_semiLink2idyn(beta_semiLink,input_sensor.getH(),beta_idyn);
    
    //insert sensor sublink
    //get sensor sublink from iDynSensor
    sub_chain.pushLink(*(&iDynLink(param_conv_idyn2mass(beta_idyn),param_conv_idyn2com(beta_idyn),param_conv_idyn2inertia(beta_idyn),0,0,0,0,input_chain[sensor_link_index].getMin(),input_chain[sensor_link_index].getMax())));
    sub_chain[0].setAngPosVelAcc(input_chain[sensor_link_index].getAng(),input_chain[sensor_link_index].getDAng(),input_chain[sensor_link_index].getD2Ang());
    
    for(int i=sensor_link_index+1;i <= final_link_index; i++) {
        sub_chain.pushLink(iDynLink(input_chain[i]));
    }
    
    sub_chain.prepareNewtonEuler();
    sub_chain.initKinematicNewtonEuler(input_sensor.getAngVel(),input_sensor.getAngAcc(),input_sensor.getLinAcc());
    *//*w0,dw0,ddp0*/
    /*
    sub_chain.computeKinematicNewtonEuler();
    return true;
}

iDynChain iDynChainExtractSubChain(iDynChain & input_chain, iDynSensor & input_sensor)
{
	iDynChain sub_chain;
	iDynChainExtractSubChain(input_chain,input_sensor,sub_chain);
	return sub_chain;
}
*/
