// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
//

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Random.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <iCub/ctrl/math.h>

#include <iCub/learningMachine/IParameterLearner.h>
#include <iCub/learningMachine/MultiTaskLinearGPRLearner.h>
#include <iCub/learningMachine/MultiTaskLinearFixedParameters.h>

#include <gsl/gsl_math.h>

#include <iostream>

#include <cassert>

#include <string>

#include "MatVetIO.h"
#include "onlineMean.h"

using namespace iCub::ctrl;
using namespace yarp::sig;
using namespace yarp::math;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::learningmachine;

Vector abs(Vector vec) {
    for(size_t i = 0; i < vec.size(); i++ ) {
        vec[i] = fabs(vec[i]);
    }
    return vec;
}

int main(int argc, char ** argv)
{   
    Matrix allRegr;
    Vector allFT;
    
    Matrix static_identifiable_parameters;
    Vector cad_parameters;
    
    Vector ftStdDev;
    
    onlineMean<Vector> cad_errors;
    onlineMean<Vector> errors;
    
    unsigned num_samples, training_samples;
    
    bool set_weight_prior = false;
    bool set_noise_scaling = false;
    
    ftStdDev = Vector(6);
    ftStdDev[0] = 0.2;
    ftStdDev[1] = 0.2;
    ftStdDev[2] = 0.2;
    ftStdDev[3] = 0.01;
    ftStdDev[4] = 0.01;
    ftStdDev[5] = 0.005;
    
    
    Property params;
    params.fromCommand(argc, argv);
    
    if( params.check("help") ) {
        std::cout << "Run in directory with data produced by inertiaObserver --dump_static" << std::endl;
        return 0;
    }
    
    if( params.check("set_weight_prior") ) {
        set_weight_prior = true;
    }
    
    if( params.check("set_noise_scaling") ) {
        set_noise_scaling = true;
    }
    
    Matrix_read("staticRegr_right_arm.ymt",allRegr);
    Vector_read("staticFT_right_arm.yvc",allFT);
    Matrix_read("staticIdentiableParameters_right_arm.ymt",static_identifiable_parameters);
    Vector_read("cad_parameters_right_arm.yvc",cad_parameters);
    
    Matrix test = eye(2,2);
    Matrix test2;
    
    Matrix_write("test_matrix_lala.ymt",test);
    Matrix_read("test_matrix_lala.ymt",test2);
    
    std::cout << "Wrote matrix: " << std::endl << test.toString() << std::endl;
    std::cout << "Read matrix   " << std::endl << test2.toString() << std::endl;
    
    test2.resize(3,3);
    
    std::cout << "Resized matrix " << std::endl << test2.toString() << std::endl;
    
    assert(allRegr.rows() == allFT.size());
    
    num_samples = allRegr.rows()/6;
    training_samples = 3*num_samples/4;
    
    std::cout << "Read " << allRegr.rows()/6 << " data samples " << std::endl;
    std::cout << "Using " << training_samples << " for training " << std::endl;
    std::cout << "Using " << num_samples-training_samples << " for testing  " << std::endl;

    
    IParameterLearner * param_learner, * offset_cad_learner ;
            
    //~~~~~~~~~~~
    param_learner = new MultiTaskLinearGPRLearner(allRegr.cols(),6);
    param_learner->setName("RLS");
    if( set_weight_prior ) {
        param_learner->setNoiseStandardDeviation(ftStdDev);
    }
    if( set_noise_scaling ) {
        param_learner->setWeightsStandardDeviation(Vector(static_identifiable_parameters.cols()+6,1.0));
    }
    
    //~~~~~~~~~~~
    offset_cad_learner = new MultiTaskLinearGPRLearner(6,6);
    offset_cad_learner->setName("OffsetLearner");
    if( set_noise_scaling ) {
        offset_cad_learner->setNoiseStandardDeviation(ftStdDev);
    }

    size_t i;
    
    //std::cout << "allRegr          " << allRegr.rows() << " " << allRegr.cols() << std::endl;
    //std::cout << "allRegr          " << allRegr.toString() << std::endl;
    
    for(i = 0; i < training_samples; i++ ) {
        //~~~~~~~~~~
        //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
        std::cout << "~~~~~~~~~~~~~~~~~I : " << i << std::endl;
        //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
        Matrix regr, regr_no_offset;
        Vector FT;
        regr = allRegr.submatrix(6*i,6*i+6-1,0,allRegr.cols()-1);
        FT = allFT.subVector(6*i,6*i+6-1);
        regr_no_offset = regr.submatrix(0,regr.rows()-1,0,regr.cols()-1-6);
        param_learner->feedSample(regr,FT);
        
        std::cout << "regr 15 col" << regr.getCol(11).toString() << std::endl;

    
        //std::cout << "Regr" << regr.toString() << std::endl;
        
        //std::cout << "allRegr          " << allRegr.rows() << " " << allRegr.cols() << std::endl;
        //std::cout << "allRegr          " << allRegr.getCol(0).toString() << std::endl;
        /*
        std::cout << "regr_no_offset" << regr_no_offset.rows() << " " << regr_no_offset.cols() << std::endl;
        std::cout << "sip           " <<  static_identifiable_parameters.rows() << " " << static_identifiable_parameters.cols() << std::endl;
        std::cout << "cad_parameters " << cad_parameters.size() << std::endl;
        std::cout.flush();
        */
        //std::cout << "regr 1 col" << regr.getCol(0).toString() << std::endl;

        
        Vector FTcad = regr_no_offset*static_identifiable_parameters.transposed()*cad_parameters;
        
        Vector res = FT-FTcad;
        
        offset_cad_learner->feedSample(eye(6,6),res);
    }
    
    Vector offset_cad;
    offset_cad = offset_cad_learner->getParameters();
    
    Vector learned_param = param_learner->getParameters();
    
    Vector learned_param_oneshot;
    
    learned_param_oneshot = pinv(allRegr,1e-7)*allFT;
    
    std::cout << allRegr.getCol(16).toString() << std::endl;
    std::cout << "Learned param offset   " << learned_param.toString() << std::endl;
    std::cout << "Cad parameters         " << (static_identifiable_parameters.transposed()*cad_parameters).toString() << std::endl;
    std::cout << "CAD learned offset     " << offset_cad.toString() << std::endl;
    std::cout << "Learned oneshot offset " << learned_param_oneshot.toString() << std::endl;

    
    for( ; i < num_samples; i++ ) {
        std::cout << "~~~~~~~~~~~~~~~~~I : " << i << std::endl;
        Prediction pred;
        Matrix regr, regr_no_offset;
        Vector FT, predFT, predFTCAD;
        regr = allRegr.submatrix(6*i,6*i+6-1,0,allRegr.cols()-1);
        regr_no_offset = regr.submatrix(0,regr.rows()-1,0,regr.cols()-1-6);
        FT = allFT.subVector(6*i,6*i+6-1);
        
        pred = param_learner->predict(regr);
        predFT = pred.getPrediction();
        
    
        
        predFTCAD = regr_no_offset*static_identifiable_parameters.transposed()*cad_parameters + offset_cad;
        
        
        std::cout << "regr 16 col" << regr.getCol(16).toString() << std::endl;
        /*
        std::cout << "FT        " << FT.toString() << std::endl;
        std::cout << "predFT    " << predFT.toString() << std::endl;
        std::cout << "predFTCAD " << predFTCAD.toString() << std::endl;
        */
        
        errors.feedSample(abs(predFT-FT));
        //std::cout << "Submitted learned error " << abs(predFT-FT).toString() << std::endl;
        cad_errors.feedSample(abs(predFTCAD-FT));
        //std::cout << "Submitted cad error " << abs(predFTCAD-FT).toString() << std::endl;

    }
    
    
    std::cout << "Errors for learned parameters " << errors.getMean().toString() << std::endl;
    std::cout << "Errors for CAD     parameters " << cad_errors.getMean().toString() << std::endl;

    return 0;
}



