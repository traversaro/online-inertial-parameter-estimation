/*
 * Copyright (C) 2007-2012 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#ifndef LM_IFIXEDSIZEMATRIXINPUTLEARNER__
#define LM_IFIXEDSIZEMATRIXINPUTLEARNER__

#include <yarp/sig/Matrix.h>
#include "iCub/learningMachine/IMachineMatrixInputLearner.h"


namespace iCub {
namespace learningmachine {

/**
 * \ingroup icub_libLM_learning_machines
 *
 * An generalized interface for a learning machine with a fixed domain
 * and codomain size, with a Matrix input. Many machine learning techniques will fall in this
 * category.
 *
 * The learning machine can be used for regression and classification
 * from R^{M \times P} to R^N. M, N and P are here fixed sizes which should be respected
 * by the samples that are fed into the system.
 *
 * \see iCub::learningMachine::IMachineLearner
 * \see iCub::learningMachine::IFixedSizeLearner
 *
 * \author Silvio Traversaro
 *
 */
class IFixedSizeMatrixInputLearner : public IMachineMatrixInputLearner {
protected:
    /**
     * The dimensionality of the input domain.
     */
    unsigned int domainRows;
    unsigned int domainCols;

    /**
     * The dimensionality of the output domain (codomain).
     */
    unsigned int coDomainSize;

    /**
     * Checks whether the input is of the desired dimensionality.
     *
     * @param input a sample input
     * @return true if the dimensionality is correct
     */
    virtual bool checkDomainSize(const yarp::sig::Matrix& input);

    /**
     * Checks whether the output is of the desired dimensionality.
     *
     * @param output a sample output
     * @return true if the dimensionality is correct
     */
    virtual bool checkCoDomainSize(const yarp::sig::Vector& output);

    /**
     * Validates whether the input and output are of the desired dimensionality. An
     * exception will be thrown if this is not the case.
     *
     * @param input a sample input
     * @param output the corresponding output
     */
    void validateDomainSizes(const yarp::sig::Matrix& input, const yarp::sig::Vector& output);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void writeBottle(yarp::os::Bottle& bot);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void readBottle(yarp::os::Bottle& bot);

public:
    /**
     * Constructor.
     *
     * @param domRows the initial domain rows
     * @param domCols the initial domain cols
     * @param cod the initial codomain size
     */
    IFixedSizeMatrixInputLearner(unsigned int domRows = 1, unsigned int domCols = 1, unsigned int cod = 1) : domainRows(domRows), domainCols(domCols), coDomainSize(cod) { }

    /*
     * Inherited from IMachineLearner, not used
     */
    virtual void feedSample(const yarp::sig::Vector& input, const yarp::sig::Vector& output);
    
    /*
     * Inherited from IMachineMatrixInputLearner, not used
     *
     */
     virtual void feedSample(const yarp::sig::Matrix& input, const yarp::sig::Vector& output);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void train();
    
    /*
     * Inherited from IMachineLearner, not used
     */
    virtual Prediction predict(const yarp::sig::Vector& input);
    
    /*
     * Inherited from IMachineMatrixInputLearner
     */
    virtual Prediction predict(const yarp::sig::Matrix& input) = 0;

    /**
     * Returns the rows of the input domain.
     *
     * @return the rows of the input domain
     */
    unsigned int getDomainRows() { return this->domainRows; }
    
    /**
     * Returns the cols of the input domain.
     *
     * @return the cols of the input domain
     */
    unsigned int getDomainCols() { return this->domainCols; }

    /**
     * Returns the size (dimensionality) of the output domain (codomain).
     *
     * @return the size of the codomain
     */
    unsigned int getCoDomainSize() { return this->coDomainSize; }

    /**
     * Mutator for the domain rows.
     *
     * @param rows the desired domain rows
     */
    virtual void setDomainRows(unsigned int rows) { this->domainRows = rows; }
    
    /**
     * Mutator for the domain cols.
     *
     * @param cols the desired domain cols
     */
    virtual void setDomainCols(unsigned int cols) { this->domainCols = cols; }

    /**
     * Mutator for the codomain size.
     *
     * @param size the desired codomain size
     */
    virtual void setCoDomainSize(unsigned int size) {this->coDomainSize = size; }

    /**
     * Inherited from IMachineLearner.
     */
    virtual std::string getInfo();

    /*
     * Inherited from IMachineLearner.
     */
    virtual std::string getConfigHelp();

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(yarp::os::Searchable& config);
    
};

} // learningmachine
} // iCub

#endif
