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


#include "iCub/learningMachine/MultiTaskLinearGPRLearner.h"

namespace iCub {
namespace learningmachine {
    
//Note, assuming in the code that Sigma_n is diagonal
    
MultiTaskLinearGPRLearner::MultiTaskLinearGPRLearner(unsigned int p, unsigned int m) {
    this->setName("MultiTaskLinearGPR");
    this->sampleCount = 0;
    
    this->setDomainRows(m);
    this->setDomainCols(p);
    
    this->setCoDomainSize(m);
    
    this->no_output_error = true;
    this->weight_prior_indefinite = true;
    
    this->reset();
}
    
MultiTaskLinearGPRLearner::MultiTaskLinearGPRLearner(const MultiTaskLinearGPRLearner& other)
  : IParameterLearner(other), sampleCount(other.sampleCount), R(other.R),
    b(other.b), w(other.w), A(other.A), inv_Sigma_n(other.inv_Sigma_n), 
    inv_Sigma_w(other.inv_Sigma_w), no_output_error(other.no_output_error),
    weight_prior_indefinite(other.weight_prior_indefinite), A_not_full_rank(other.A_not_full_rank) {
}

MultiTaskLinearGPRLearner::~MultiTaskLinearGPRLearner() {
}

MultiTaskLinearGPRLearner& MultiTaskLinearGPRLearner::operator=(const MultiTaskLinearGPRLearner& other) {
    if(this == &other) return *this; // handle self initialization

    this->IFixedSizeMatrixInputLearner::operator=(other);
    this->sampleCount = other.sampleCount;

    this->R = other.R;
    this->b = other.b;
    this->w = other.w;
    this->A = other.A;
    
    this->no_output_error = other.no_output_error;
    this->weight_prior_indefinite = other.weight_prior_indefinite;
    this->A_not_full_rank= other.A_not_full_rank;
    
    return *this;
}


void MultiTaskLinearGPRLearner::feedSample(const yarp::sig::Matrix& input_matrix, const yarp::sig::Vector& output) {
    this->IFixedSizeMatrixInputLearner::feedSample(input_matrix, output);
   
    //update R (or, until it is full rank, A)
    if( ! this->A_not_full_rank) {
        //update R
           if( ! this->no_output_error ) {
            for(int i = 0; i < this->getDomainRows(); i++ ) {
                //diagonal assumption
                cholupdate(this->R, sqrt(inv_Sigma_n(i,i))*input_matrix.getRow(i));
            }
        } else {
            for(int i = 0; i < this->getDomainRows(); i++ ) {
                //diagonal assumption
                cholupdate(this->R, input_matrix.getRow(i));
            }
        }
 
    } else {
        assert(this->A_not_full_rank);
        if( this->no_output_error ) {
            //update A
            this->A += input_matrix.transposed()*input_matrix;
            
        } else {

            assert(this->weight_prior_indefinite);
            //Update A
            this->A += input_matrix.transposed()*inv_Sigma_n*input_matrix;
        }
        //check if after the update, A became of full rank
        if( isfullrank(this->A) ) {
                this->R = choldecomp(A);
                this->A_not_full_rank= false;
        } 
    }
    
    
    //update b
    if( !this->no_output_error ) {
    
        this->b += input_matrix.transposed()*inv_Sigma_n*output;
        
    } else {
        
        this->b = this->b + input_matrix.transposed()*output;
        
    }
    
    //update w 
    if( ! this->A_not_full_rank) {
        cholsolve(this->R, this->b, this->w);
    } else {
        //\todo
        //would be a better idea to implement the same tolerance heuristics in pinv
        this->w = yarp::math::pinv(this->A,1e-5)*this->b;
    }

    this->sampleCount++;

}

void MultiTaskLinearGPRLearner::train() {
};

Prediction MultiTaskLinearGPRLearner::predict(const yarp::sig::Vector& input) {
    return Prediction(input);
}

Prediction MultiTaskLinearGPRLearner::predict(const yarp::sig::Matrix& input) {
    this->checkDomainSize(input);

    yarp::sig::Vector output = (input * this->w);
    
    
    //Returning stddeviation, so calculating the diagonal of prediction 
    //covariance matrix and taking the square root of it
    yarp::sig::Vector std;
    yarp::sig::Matrix covMatrix;
    //strange definition of cholsolve regarding rows/columns, check it out
    covMatrix = input*(cholsolve(this->R,input).transposed());
    
    std = map(diagonal(covMatrix),sqrt);
    
    return Prediction(output,std);

}

void MultiTaskLinearGPRLearner::reset() {
    this->sampleCount = 0;
    if( this->no_output_error ) {
        this->A_not_full_rank = true;
        this->A = zeros(this->getDomainCols(), this->getDomainCols());
        this->R = zeros(this->getDomainCols(), this->getDomainCols());
        this->b = zeros(this->getDomainCols());
        this->w = zeros(this->getDomainCols());
    } else if ( this->weight_prior_indefinite ) {
        this->A_not_full_rank = true;
        this->A = zeros(this->getDomainCols(), this->getDomainCols());
        this->R = zeros(this->getDomainCols(), this->getDomainCols());
        this->b = zeros(this->getDomainCols());
        this->w = zeros(this->getDomainCols());
    } else {
        //Default case
        this->A_not_full_rank = false;
        //Ugly, but for diagonal matrix and only at reset should not give problems
        this->R = choldecomp(inv_Sigma_w); 
        this->b = zeros(this->getDomainCols());
        this->w = zeros(this->getDomainCols());
    }
}

std::string MultiTaskLinearGPRLearner::getInfo() {
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

std::string MultiTaskLinearGPRLearner::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeMatrixInputLearner::getConfigHelp();
    return buffer.str();
}

void MultiTaskLinearGPRLearner::writeBottle(yarp::os::Bottle& bot) {
    bot << this->R << this->b << this->w << this->A << this->inv_Sigma_n << this->inv_Sigma_w << this->no_output_error << this->weight_prior_indefinite << this->A_not_full_rank << this->sampleCount;
    // make sure to call the superclass's method
    this->IFixedSizeMatrixInputLearner::writeBottle(bot);
}

void MultiTaskLinearGPRLearner::readBottle(yarp::os::Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeMatrixInputLearner::readBottle(bot);
    bot >> this->sampleCount >> this->A_not_full_rank >> this->weight_prior_indefinite >> this->no_output_error >> this->inv_Sigma_w >> this->inv_Sigma_n >> this->A >> this->w >> this->b >> this->R;
}

void MultiTaskLinearGPRLearner::setNoiseStandardDeviation(double s) {
    yarp::sig::Vector n_sd(this->getDomainRows(),s);
    setNoiseStandardDeviation(n_sd);
}

void MultiTaskLinearGPRLearner::setNoiseStandardDeviation(const yarp::sig::Vector& s) {
    if( s.size() != this->getDomainRows() ) {
        throw std::runtime_error("MultiTaskLinearGPRLearner: wrong dimension of noise std deviation");
    }
    
    inv_Sigma_n = zeros(this->getDomainRows(),this->getDomainRows());
    
    for(unsigned i=0; i < s.size(); i++ ) {
        inv_Sigma_n(i,i) = 1/(s[i]*s[i]);
    }
    
    this->no_output_error = false;
    
    this->reset();
    
}

yarp::sig::Vector MultiTaskLinearGPRLearner::getNoiseStandardDeviation() {
    yarp::sig::Vector ret(this->getDomainRows());
    for(unsigned i=0; i < ret.size(); i++ ) {
        ret[i] = sqrt(1/inv_Sigma_n(i,i));
    }
    return ret;
}
    
void MultiTaskLinearGPRLearner::setWeightsStandardDeviation(double s) {
    yarp::sig::Vector sd(this->getDomainCols(),s);
    setNoiseStandardDeviation(sd);
}

void MultiTaskLinearGPRLearner::setWeightsStandardDeviation(const yarp::sig::Vector& s) {
if( s.size() != this->getDomainCols() ) {
        throw std::runtime_error("MultiTaskLinearGPRLearner: wrong dimension of noise std deviation");
    }
    
    inv_Sigma_w = zeros(this->getDomainCols(),this->getDomainCols());
    
    for(unsigned i=0; i < s.size(); i++ ) {
        inv_Sigma_w(i,i) = 1/(s[i]*s[i]);
    }
    
    this->weight_prior_indefinite = false;
    
    this->reset();
}

yarp::sig::Vector MultiTaskLinearGPRLearner::getWeightsStandardDeviation() {
    yarp::sig::Vector ret(this->getDomainRows());
    for(unsigned i=0; i < ret.size(); i++ ) {
        ret[i] = sqrt(1/inv_Sigma_w(i,i));
    }
    return ret;
}

bool MultiTaskLinearGPRLearner::configure(yarp::os::Searchable& config) {
    throw std::runtime_error("MultiTaskLinearGPRLearner: configure call not implemented");
}

yarp::sig::Vector MultiTaskLinearGPRLearner::getParameters() const {
    //return yarp::sig::Vector(0);
    return this->w;
}

/*
yarp::sig::Vector MultiTaskLinearGPRLearner::saveParameters(const string file_name) const {
}
*/ 

/**
	
MultiTaskLinearGPRLearner::MultiTaskLinearGPRLearner(unsigned int dom, unsigned int cod, double sigma) {
	yarp::sig::Vector sigma_vec(cod,sigma);
	MultiTaskLinearGPRLearner(dom,cod,sigma_vec);
}


MultiTaskLinearGPRLearner::MultiTaskLinearGPRLearner(unsigned int dom, unsigned int cod, yarp::sig::Vector sigma) {
    this->setName("MultiTaskLinearGPR");
    this->sampleCount = 0;
    // make sure to not use initialization list to constructor of base for
    // domain and codomain size, as it will not use overloaded mutators
    this->setDomainSize(dom);
    // slightly inefficient to use mutators, as we are initializing multiple times
    this->setCoDomainSize(cod);
    this->setSigma(sigma);
}

MultiTaskLinearGPRLearner::MultiTaskLinearGPRLearner(const MultiTaskLinearGPRLearner& other)
  : IFixedSizeLearner(other), sampleCount(other.sampleCount), R(other.R),
    b(other.b), w(other.w), sigma(other.sigma) {
}

MultiTaskLinearGPRLearner::~MultiTaskLinearGPRLearner() {
}

MultiTaskLinearGPRLearner& MultiTaskLinearGPRLearner::operator=(const MultiTaskLinearGPRLearner& other) {
    if(this == &other) return *this; // handle self initialization

    this->IFixedSizeLearner::operator=(other);
    this->sampleCount = other.sampleCount;

    this->R = other.R;
    this->b = other.b;
    this->w = other.w;
    this->sigma = other.sigma;

    return *this;
}

bool MultiTaskLinearGPRLearner::checkInputSize(const yarp::sig::Matrix& input_feature_matrix) {
    return (input_feature_matrix.rows() == this->getDomainSize()) && (input_feature_matrix.cols() == this->getCoDomainSize());
}

void MultiTaskLinearGPRLearner::validateDomainSizes(const yarp::sig::Matrix& input_feature_matrix, const yarp::sig::Vector& output) {
    if(!this->checkInputSize(input_feature_matrix)) {
        throw std::runtime_error("Input feature matrix sample has invalid dimensionality");
    }
    if(!this->checkCoDomainSize(output)) {
        throw std::runtime_error("Output sample has invalid dimensionality");
    }
}

void MultiTaskLinearGPRLearner::feedSample(const yarp::sig::Vector& input, const yarp::sig::Vector& output) {
    this->IFixedSizeLearner::feedSample(input, output);
    throw std::runtime_error("MultiTaskLinearGPRLearner: an input matrix is required for feedSample");
}

void MultiTaskLinearGPRLearner::feedSample(const yarp::sig::Matrix& input_feature_matrix, const yarp::sig::Vector& output) {
    //this->IFixedSizeLearner::feedSample(input, output);
    this->validateDomainSizes(input_feature_matrix,output);
	
    // update R
    for(int i; i < this->getDomainSize(); i++ ) {
		cholupdate(this->R, (1/this->sigma[i])*input_feature_matrix.getCol(i));
	}

    // update b
    this->b = this->b + input_feature_matrix*diagonal(this->inv_sqr_sigma)*output;

    // update w
    cholsolve(this->R, this->b, this->w);

    this->sampleCount++;
}

void MultiTaskLinearGPRLearner::train() {

}

Prediction MultiTaskLinearGPRLearner::predict(const yarp::sig::Vector& input_feature_matrix) {
    this->checkInputSize(input);

    yarp::sig::Vector output = (input_feature_matrix).transposedd * this->w;

	//todo variance calculation
    //it should be the diagonal of input_feature_matrix.transposed * inv(A) * input_feature_matrix with inv A calculated through cholesky

    return Prediction(output);
}

void MultiTaskLinearGPRLearner::reset() {
    this->sampleCount = 0;
    this->R = eye(this->getDomainSize(), this->getDomainSize())
    this->b = zeros(this->getDomainSize());
    this->w = zeros(this->getDomainSize());
}

std::string MultiTaskLinearGPRLearner::getInfo() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getInfo();
    buffer << "Sigma: " << this->getSigma() << " | ";
    buffer << "Sample Count: " << this->sampleCount << std::endl;
    //for(unsigned int i = 0; i < this->machines.size(); i++) {
    //    buffer << "  [" << (i + 1) << "] ";
    //    buffer << "lambda: " << this->machines[i]->getLambda();
    //    buffer << std::endl;
    //}
    return buffer.str();
}

std::string MultiTaskLinearGPRLearner::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getConfigHelp();
    buffer << "  sigma val             Signal noise sigma" << std::endl;
    return buffer.str();
}

void MultiTaskLinearGPRLearner::writeBottle(yarp::os::Bottle& bot) {
    bot << this->R << this->b << this->w << this->sigma << this->sampleCount;
    // make sure to call the superclass's method
    this->IFixedSizeLearner::writeBottle(bot);
}

void MultiTaskLinearGPRLearner::readBottle(yarp::os::Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeLearner::readBottle(bot);
    bot >> this->sampleCount >> this->sigma >> this->w >> this->b >> this->R;
}

void MultiTaskLinearGPRLearner::setDomainSize(unsigned int size) {
    this->IFixedSizeLearner::setDomainSize(size);
    this->reset();
}

void MultiTaskLinearGPRLearner::setCoDomainSize(unsigned int size) {
    this->IFixedSizeLearner::setCoDomainSize(size);
    this->reset();
}

void MultiTaskLinearGPRLearner::setSigma(const yarp::sig::Vector& s) {
	if(!this->checkCoDomainSize(s)) {
        throw std::runtime_error("Sigma vector has invalid dimensionality");
    }
	
	for( int i=0; i < s.size(); i++ ) {
		if( s[i] <= 0.0 ) {
			throw std::runtime_error("Signal noise sigma has to be larger than 0");
			return; //Make sense?
		}
	}
	 
    this->sigma = s;
    this->inv_sqr_sigma = Vector(s.size());
    for(int i=0;i<this->inv_sqr_sigma.size(); i++) {
		this->inv_sqr_sigma[i] = 1/(s[i]*s[i]);
	}
    
    this->reset();
}

yarp::sig::Vector MultiTaskLinearGPRLearner::getSigma() {
    return this->sigma;
}


bool MultiTaskLinearGPRLearner::configure(yarp::os::Searchable& config) {
    bool success = this->IFixedSizeLearner::configure(config);

    // format: set sigma val
    if(config.find("sigma").isDouble() || config.find("sigma").isInt()) {
        this->setSigma(config.find("sigma").asDouble());
        success = true;
    }

    return success;
}
**/
} // learningmachine
} // iCub


