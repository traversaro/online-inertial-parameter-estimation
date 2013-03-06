/*
 * Copyright (C) 2007-2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Silvio Traversaro
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

#include <stdexcept>
#include <sstream>

#include "iCub/learningMachine/IFixedSizeMatrixInputLearner.h"

namespace iCub {
namespace learningmachine {


void IFixedSizeMatrixInputLearner::feedSample(const yarp::sig::Vector& input, const yarp::sig::Vector& output) {
    throw std::runtime_error("IFixedSizeMatrixInputLearner: the input sample must be a Matrix");
}

void IFixedSizeMatrixInputLearner::feedSample(const yarp::sig::Matrix& input, const yarp::sig::Vector& output) {
    this->validateDomainSizes(input,output);
}


void IFixedSizeMatrixInputLearner::train() {
}

Prediction IFixedSizeMatrixInputLearner::predict(const yarp::sig::Vector& input) {
    throw std::runtime_error("IFixedSizeMatrixInputLearner: the input sample must be a Matrix");
}

bool IFixedSizeMatrixInputLearner::configure(yarp::os::Searchable& config) {
    bool success = false;
    // set the domain rows (int)
    if(config.find("domRows").isInt()) {
        this->setDomainRows(config.find("domRows").asInt());
        success = true;
    }
    // set the domain cols (int)
    if(config.find("domCols").isInt()) {
        this->setDomainCols(config.find("domCols").asInt());
        success = true;
    }
    
    // set the codomain size (int)
    if(config.find("cod").isInt()) {
        this->setCoDomainSize(config.find("cod").asInt());
        success = true;
    }

    return success;
}

bool IFixedSizeMatrixInputLearner::checkDomainSize(const yarp::sig::Matrix& input) {
    return (input.cols() == this->getDomainCols()) && (input.rows() == this->getDomainRows());
}

bool IFixedSizeMatrixInputLearner::checkCoDomainSize(const yarp::sig::Vector& output) {
    return (output.size() == this->getCoDomainSize());
}

void IFixedSizeMatrixInputLearner::validateDomainSizes(const yarp::sig::Matrix& input, const yarp::sig::Vector& output) {
    if(!this->checkDomainSize(input)) {
        throw std::runtime_error("Input sample has invalid dimensionality");
    }
    if(!this->checkCoDomainSize(output)) {
        throw std::runtime_error("Output sample has invalid dimensionality");
    }
}

void IFixedSizeMatrixInputLearner::writeBottle(yarp::os::Bottle& bot) {
    bot.addInt(this->getDomainRows());
    bot.addInt(this->getDomainCols());
    bot.addInt(this->getCoDomainSize());
}

void IFixedSizeMatrixInputLearner::readBottle(yarp::os::Bottle& bot) {
    this->setCoDomainSize(bot.pop().asInt());
    this->setDomainCols(bot.pop().asInt());
    this->setDomainRows(bot.pop().asInt());
}

std::string IFixedSizeMatrixInputLearner::getInfo() {
    std::ostringstream buffer;
    buffer << this->IMachineLearner::getInfo();
    buffer << "Domain rows: " << this->getDomainRows() << std::endl;
    buffer << "Domain cols: " << this->getDomainCols() << std::endl;
    buffer << "Codomain size: " << this->getCoDomainSize() << std::endl;
    return buffer.str();
}

std::string IFixedSizeMatrixInputLearner::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IMachineLearner::getConfigHelp();
    buffer << "  domRows rows              Domain cols" << std::endl;
    buffer << "  domCols cols              Domain cols" << std::endl;
    buffer << "  cod size                  Codomain size" << std::endl;
    return buffer.str();
}

/*
Prediction predict(const yarp::sig::Vector& input) {
    //throw std::runtime_error("IFixedSizeMatrixInputLearner: the input sample must be a Matrix");
}
*/

} // learningmachine
} // iCub
