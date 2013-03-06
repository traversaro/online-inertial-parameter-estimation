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
#include <algorithm>
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

// random generator function:
ptrdiff_t myrandom (ptrdiff_t i) { return  (ptrdiff_t) yarp::os::Random::uniform(0,(int)i); }

// pointer object to it:
ptrdiff_t (*p_myrandom)(ptrdiff_t) = myrandom;

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
    
    int num_samples, training_samples;
    
    onlineMean<Vector> cad_errors;
    onlineMean<Vector> errors;
    
    onlineMean<Vector> cad_errors_norms;
    onlineMean<Vector> errors_norms;
    
    std::string regr_file, cad_param_file, measure_file, identifiable_param_file;
    double training_percentage = 0.5;
    
    
    bool set_weight_prior = false;
    bool set_noise_scaling = true;
    
    bool using_different_set_for_validation = false;
    
    ftStdDev = Vector(6);
    ftStdDev[0] = 0.0759966;
    ftStdDev[1] = 0.0893956;
    ftStdDev[2] = 0.1793871;
    ftStdDev[3] = 0.0019388; 
    ftStdDev[4] = 0.0028635; 
    ftStdDev[5] = 0.0014280;
    
    
    regr_file = "staticRegr_right_arm.ymt";
    measure_file = "staticFT_right_arm.yvc";
    identifiable_param_file = "staticIdentiableParameters_right_arm.ymt";
    cad_param_file = "cad_parameters_right_arm.yvc";
    
    
    Property params;
    params.fromCommand(argc, argv);
    
    if( params.check("help") ) {
        std::cout << "Run in directory with data produced by inertiaObserver --dump_static" << std::endl;
        std::cout << "for other options, check the source code :-) " << std::endl;
        return 0;
    }
    
    if( params.check("set_weight_prior") ) {
        set_weight_prior = true;
    }
    
    if( params.check("set_noise_scaling") ) {
        set_noise_scaling = true;
    }
    
    if( params.check("regr" ) ) {
        std::cout << "Found regr paramter" << std::endl;
        regr_file = params.find("regr").asString();
    }
    
    if( params.check("measure" ) ) {
        std::cout << "Found measure paramter" << std::endl;
        measure_file = params.find("measure").asString();
    }
    
    if( params.check("cad_param" ) ) {
        std::cout << "Found cad_param parameter" << std::endl;
        regr_file = params.find("cad_param").asString();
    }
    
    if( params.check("identifiable_param" ) ) {
        std::cout << "Found identifiable_param paramter" << std::endl;
        identifiable_param_file = params.find("identifiable_param").asString();
    }
    
    if( params.check("training") ) {
        double perc;
        std::cout << "Found training parameter" << std::endl; 
        perc = params.find("training").asDouble();
        if( perc < 1.0 && perc > 0.0 ) {
            training_percentage = perc;
        } else {
            std::cout << "training percentage should be between 0.0 and 1.0, not " << perc << std::endl;
        }
    }
    
    std::string validation_regr_file, validation_measure_file;
    
    Matrix allRegr_validation;
    Vector allFT_validation;
    
    if( params.check("validation_regr") || params.check("validation_measure") ) {
        using_different_set_for_validation = true;
        validation_regr_file  = params.find("validation_regr").asString();
        validation_measure_file = params.find("validation_measure").asString();
    }

    

    bool res = true;
    res &= Matrix_read(regr_file,allRegr);
    res &= Vector_read(measure_file,allFT);
    res &= Matrix_read(identifiable_param_file,static_identifiable_parameters);
    res &= Vector_read(cad_param_file,cad_parameters);
    
    if( using_different_set_for_validation ) { 
        res &= Matrix_read(validation_regr_file,allRegr_validation);
        res &= Vector_read(validation_measure_file,allFT_validation);
    }

    if( !res ) {
        std::cout << "Input error " << std::endl;
        return 0;
    }

    assert(allRegr.rows() == (int)allFT.size());
    
    num_samples = allRegr.rows()/6;
    
    if( !using_different_set_for_validation ) { 
        training_samples = (int)(training_percentage*(double)num_samples);
    } else {
        training_samples = num_samples;
    }
    
    std::cout << "Read " << allRegr.rows()/6 << " data samples " << std::endl;
    
    

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
    
    yarp::os::Random::seed(yarp::os::Time::now());
    
    std::vector<int> validation_samples;
    
    std::vector<int> scrambled_indexes;
    
    int l;
    for(l = 0; l < num_samples; l++ ) { scrambled_indexes.push_back(l); }
    
    std::random_shuffle ( scrambled_indexes.begin(), scrambled_indexes.end(), p_myrandom);
    
    for(l = 0; l < training_samples; l++ ) {
        i = scrambled_indexes[l];
        //std::cout << "~~~~~~~~~~~~~~~~ I " << i << std::endl;
        Matrix regr, regr_no_offset;
        Vector FT;
        regr = allRegr.submatrix(6*i,6*i+6-1,0,allRegr.cols()-1);
        FT = allFT.subVector(6*i,6*i+6-1);
        regr_no_offset = regr.submatrix(0,regr.rows()-1,0,regr.cols()-1-6);
        
        //std::cout << "FT " << FT.toString() << std::endl;
        //std::cout << "regr " << regr.toString() << std::endl;
        
        //training
        param_learner->feedSample(regr,FT);
        Vector FTcad = regr_no_offset*static_identifiable_parameters.transposed()*cad_parameters;
        Vector res = FT-FTcad;
        offset_cad_learner->feedSample(eye(6,6),res);

    }
    
    std::cout << "Using " << training_samples  << " for training " << std::endl;
    
    if( !using_different_set_for_validation ) {
        std::cout << "Using " << num_samples - training_samples << " for testing  " << std::endl;
    } else {
        std::cout << "Using " << allFT_validation.size()/6 << " for validation " << std::endl;
    }

    Vector offset_cad = offset_cad_learner->getParameters();
    Vector learned_param = param_learner->getParameters();
    
    
    if( !using_different_set_for_validation ) { 
        std::cout << "Using right branch" << std::endl;
        for( ; l < num_samples ; l++ ) {
            i = scrambled_indexes[l];
            
            Prediction pred;
            Matrix regr, regr_no_offset;
            Vector FT, predFT, predFTCAD;
            Vector F,T, predF,predT,predFCAD,predTCAD;
            regr = allRegr.submatrix(6*i,6*i+6-1,0,allRegr.cols()-1);
            regr_no_offset = regr.submatrix(0,regr.rows()-1,0,regr.cols()-1-6);
            FT = allFT.subVector(6*i,6*i+6-1);
            
            F = FT.subVector(0,2);
            T = FT.subVector(3,5);
            
            pred = param_learner->predict(regr);
            predFT = pred.getPrediction();
            predF = predFT.subVector(0,2);
            predT = predFT.subVector(3,5);
            
            predFTCAD = regr_no_offset*static_identifiable_parameters.transposed()*cad_parameters + offset_cad;
            predFCAD = predFTCAD.subVector(0,2);
            predTCAD = predFTCAD.subVector(3,5);
            
            
            errors.feedSample(abs(predFT-FT));
            cad_errors.feedSample(abs(predFTCAD-FT));
            
            Vector err_norms(2), cad_err_norms(2);
            err_norms[0] = norm(predF-F);
            err_norms[1] = norm(predT-T);
            
            cad_err_norms[0] = norm(predFCAD-F);   
            cad_err_norms[1] = norm(predTCAD-T);

            errors_norms.feedSample(err_norms);
            cad_errors_norms.feedSample(cad_err_norms);
        }
    } else {
        for(i = 0; i < allFT_validation.size()/6; i++ ) {
            Prediction pred;
            Matrix regr, regr_no_offset;
            Vector FT, predFT, predFTCAD;
            regr = allRegr_validation.submatrix(6*i,6*i+6-1,0,allRegr.cols()-1);
            regr_no_offset = regr.submatrix(0,regr.rows()-1,0,regr.cols()-1-6);
            FT = allFT_validation.subVector(6*i,6*i+6-1);
            
            pred = param_learner->predict(regr);
            predFT = pred.getPrediction();
            
            predFTCAD = regr_no_offset*static_identifiable_parameters.transposed()*cad_parameters + offset_cad;
            
            errors.feedSample(abs(predFT-FT));
            cad_errors.feedSample(abs(predFTCAD-FT));
        }
    }
    
    Vector errors_force = errors.getMean().subVector(0,2);
    Vector errors_torque = errors.getMean().subVector(3,5);
    
    Vector cad_errors_force = cad_errors.getMean().subVector(0,2);
    Vector cad_errors_torque = cad_errors.getMean().subVector(3,5);

    std::cout << "Errors for learned parameters " << errors.getMean().toString() << std::endl;
    std::cout << "Errors for CAD     parameters " << cad_errors.getMean().toString() << std::endl;
    
    std::cout << "Errors for learned param " << norm(errors_force) << "\t" << norm(errors_torque) << std::endl;
    std::cout << "Errors for CAD     param " << norm(cad_errors_force) << "\t" << norm(cad_errors_torque) << std::endl;
    

    
    if( !using_different_set_for_validation ) {
        Vector std_dev(2);
        Vector std_dev_cad(2);
        std::cout << "Mean norm error " << std::endl;
        std::cout << "Mean Errors for learned param " << errors_norms.getMean().toString() << std::endl;
        std::cout << "Mean Errors for learned param " << cad_errors_norms.getMean().toString() << std::endl;
        
        std_dev = errors_norms.getVariance();
        std_dev_cad = cad_errors_norms.getVariance();
        
        std_dev[0] = sqrt(std_dev[0]);
        std_dev[1] = sqrt(std_dev[1]);
        
        std_dev_cad[0] = sqrt(std_dev_cad[0]);
        std_dev_cad[1] = sqrt(std_dev_cad[1]);

        
        std::cout << "SD Errors for learned param " << std_dev.toString() << std::endl;
        std::cout << "SD Errors for learned param " << std_dev_cad.toString() << std::endl;
    } 
    return 0;
}



