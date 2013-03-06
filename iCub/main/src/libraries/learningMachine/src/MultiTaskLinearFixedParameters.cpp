/*
 * Copyright (C) 2007-2012 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Silvio Traversaro
 * email:   pegua1@gmail.com
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <cassert>
#include <stdexcept>
#include <cmath>

#include <iostream>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include "iCub/learningMachine/RLSLearner.h"
#include "iCub/learningMachine/Math.h"
#include "iCub/learningMachine/Serialization.h"

using namespace yarp::math;
using namespace iCub::learningmachine::serialization;
using namespace iCub::learningmachine::math;


#include "iCub/learningMachine/MultiTaskLinearFixedParameters.h"

namespace iCub {
namespace learningmachine {
    
    
MultiTaskLinearFixedParameters::MultiTaskLinearFixedParameters(unsigned int p, unsigned int m, yarp::sig::Vector parameters) {
    this->setName("MultiTaskLinearFixedParameters");
    this->sampleCount = 0;
    
    this->setDomainRows(m);
    this->setDomainCols(p);
    
    this->setCoDomainSize(m);
    
    this->w = parameters;
    
    if( this->w.size() != this->getDomainCols() ) {
        throw std::runtime_error("MultiTaskLinearFixedParameters: wrong parameter size");
    }
        
    this->reset();
}
    
MultiTaskLinearFixedParameters::MultiTaskLinearFixedParameters(const MultiTaskLinearFixedParameters& other)
  : IParameterLearner(other), sampleCount(other.sampleCount), w(other.w) {
}

MultiTaskLinearFixedParameters::~MultiTaskLinearFixedParameters() {
}

MultiTaskLinearFixedParameters& MultiTaskLinearFixedParameters::operator=(const MultiTaskLinearFixedParameters& other) {
    if(this == &other) return *this; // handle self initialization

    this->IParameterLearner::operator=(other);
    this->sampleCount = other.sampleCount;

    this->w = other.w;
    
    return *this;
}


void MultiTaskLinearFixedParameters::feedSample(const yarp::sig::Matrix& input_matrix, const yarp::sig::Vector& output) {
    this->IParameterLearner::feedSample(input_matrix, output);
    
    this->sampleCount++;
}

void MultiTaskLinearFixedParameters::train() {
};


Prediction MultiTaskLinearFixedParameters::predict(const yarp::sig::Matrix& input) {
    this->checkDomainSize(input);

    yarp::sig::Vector output = (input * this->w);
    //No std deviation calculation implemented
    
    return Prediction(output);

}

void MultiTaskLinearFixedParameters::reset() {
    this->sampleCount = 0;
}

std::string MultiTaskLinearFixedParameters::getInfo() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeMatrixInputLearner::getInfo();
    buffer << "Sample Count: " << this->sampleCount << std::endl;
    //for(unsigned int i = 0; i < this->machines.size(); i++) {
    //    buffer << "  [" << (i + 1) << "] ";
    //    buffer << "lambda: " << this->machines[i]->getLambda();
    //    buffer << std::endl;
    //}
    return buffer.str();
}

std::string MultiTaskLinearFixedParameters::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeMatrixInputLearner::getConfigHelp();
    return buffer.str();
}

void MultiTaskLinearFixedParameters::writeBottle(yarp::os::Bottle& bot) {
    bot << this->w << this->sampleCount;
    // make sure to call the superclass's method
    this->IFixedSizeMatrixInputLearner::writeBottle(bot);
}

void MultiTaskLinearFixedParameters::readBottle(yarp::os::Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeMatrixInputLearner::readBottle(bot);
    bot >> this->sampleCount >> this->w;
}


bool MultiTaskLinearFixedParameters::configure(yarp::os::Searchable& config) {
    throw std::runtime_error("MultiTaskLinearFixedParameters: configure call not implemented");
}

yarp::sig::Vector MultiTaskLinearFixedParameters::getParameters() const {
    return this->w;
}

void  MultiTaskLinearFixedParameters::setParameters(const yarp::sig::Vector & parameters) {
    if( parameters.size() != this->getDomainCols() ) {
        throw std::runtime_error("MultiTaskLinearFixedParameters: wrong parameter size");
    }
    
    this->w = parameters;
    
    this->reset();
    return;
}

void MultiTaskLinearFixedParameters::setNoiseStandardDeviation(const yarp::sig::Vector& s) {
}

void MultiTaskLinearFixedParameters::setWeightsStandardDeviation(const yarp::sig::Vector& s) {
}




} // learningmachine
} // iCub


