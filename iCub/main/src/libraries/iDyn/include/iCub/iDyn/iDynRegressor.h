/* 
 * Copyright (C) 2012
 * Author: Silvio Traversaro
 * email:  
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
/**
 * \defgroup iDynRegressor iDynRegressor
 * 
 * @ingroup iDyn
 *
 * Functions for calculating regressors with respect to dynamical parameters
 * 
 * 
 * 
 * \par	
 * All the code defined in this header ignores any information regarding blocked joints
 * 
 * \par 
 * All the code assumes that no contact is occuring on the mechanical chain after the sensor
 *
 * 
 * \par 	
 * In documentation, most of the notation is the same used in Sciavicco & Siciliano
 * 			so, for example, \f$\overline{I}_i\f$ is the inertia matrix of the link \f$i\f$ with
 * 			respect to center of mass of the link, while \f$\hat{I}_i\f$  is the inertia matrix 
 * 			of the link \f$i\f$ but respect to the origin of the link frame of reference
 *
 * 
 * \par  	
 * The convention used for the vectorization of the dynamical parameters 
 * 			for one link is \f$[ \ m \ mr_x \ mr_y \ mr_z \ \hat{I}_{xx} \ \hat{I}_{xy} \ \hat{I}_{xz} \ \hat{I}_{yy} \ \hat{I}_{yz} \ \hat{I}_{zz}]\f$
 * 
 * 
 * \par 	
 * iDyn library stores the dynamical parameters for each link in this format:
 * 			mass , center of mass and inertia matrix w.r.t. center of mass. As for the identification
 * 			a nonlinear combination of this parameter are used, through the code the rappresentation used 
 * 			in the rest of the iDyn library is called "idyn" while the one used for identification is called "ident".
 * 			Appropriate function for conversion are provided. \n
 * 			Where not explicitly mentioned, it safe to assumente that a phi vector is expressed using ident notation. \n
 * 			Pay attention, as dynamical parameters of the semilink stored inside iDynSensor are expressed in another format,
 * 			also for this format appropriate conversion function are provided.
 * 
 * \section tested_os_sec Tested OS
 * 
 * Linux
 *
 *
 * \author Silvio Traversaro
 * 
 */
 
#ifndef __IDYNREGRESSOR__
#define __IDYNREGRESSOR__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

namespace iCub
{

namespace iDyn
{

/**
 * \ingroup iDynRegressor
 * 
 * Class containing all the static methods for calculating regressors 
 * 
 */
namespace Regressor
{
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //iDynLink regressors
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    /**
    * Compute the regressor matrix relating the inertial parameters of a link
    * to the net wrench acting on that link, expressed on the frame of reference of the link
    * @param p_link pointer to the structure of the link
    * @return the 6x10 regressor matrix 
    */
     yarp::sig::Matrix iDynLinkRegressorNetWrench(iCub::iDyn::iDynLink * p_link);
    
    
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //iDynChain/iDynSensor regressors
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /**
    * For a given iDynChain calculate the regressor matrix \f$Y_1\f$ such that \f${W \brack \tau} = Y_1 \phi\f$ 
    * where \f$W\f$ is the sensor wrench for the chain,
    * \f$\tau\f$ is the vector of the torques on the joints placed in the chain after the FT sensor \n
    * assuming that there is no external contact on the chain \n
    * and phi is the vector containing the inertial parameters of the chain,only for the links after the sensor \n
    * \note Equivalent iDyn function: getSensorForceMoment
    * @param p_chain pointer to the given iDynChain
    * @param p_sensor pointer to the given iDynSensor
    * @param Y reference to the matrix where store the output matrix
    * @param excluded_link optional index (referring to the original iDynChain) of a link excluded from calculation of regressor matrix (usually because it is a virtual link introduced to describe a joint with more than one DOF)
    * @return false in case of error, true otherwise
    */
     bool iDynChainRegressorSensorWrench(iCub::iDyn::iDynChain *p_chain,iCub::iDyn::iDynSensor * p_sensor, yarp::sig::Matrix & Y, const int excluded_link = -1);
    
    /**
    * For a given iDynChain calculate the regressor matrix \f$Y_1\f$ such that \f${W \brack \tau} = Y_1 \phi\f$ 
    * where \f$W\f$ is the sensor wrench for the chain (the same obtained calling getSensorForceMoment),
    * \f$\tau\f$ is the vector of the torques on the joints placed in the chain after the FT sensor \n
    * assuming that there is no external contact on the chain \n
    * and phi is the vector containing the inertial parameters of the chain,only for the links after the sensor
    * @param p_chain pointer to the given iDynChain
    * @param p_sensor pointer to the given iDynSensor
    * @param Y reference to the matrix where store the output matrix
    * @param excluded_links vector of bool of the same length of the iDynChain, if excluded_links[i] is true the link i is excluded from the calculation of the regressor matrix (usually because it is a virtual link introduced to describe a joint with more than one DOF)
    * @return false in case of error, true otherwise
    */
     bool iDynChainRegressorSensorWrench(iCub::iDyn::iDynChain *p_chain,iCub::iDyn::iDynSensor * p_sensor, yarp::sig::Matrix & Y, std::vector<bool> excluded_links);
    
    /**
     * For a given iDynChain return a vector \f$Y_i^{\top}\f$ such that \f$\tau_i = {z_0}^{\top} {{Ad}^{\top}_{H^s_{i-1}}} W_s^s + Y_i\phi\f$ \n
     * Where phi is the vector returned by iDynChainGetBeta , \f${z_0}^{\top} = [  0 \ 0 \ 0 \ 0 \ 0 \ 1 ]\f$, \f$\tau_i\f$ is \f$i\f$ torque. \n
     * In a nutshell, this regressor represent the inertial contribution to the torque calculated through the measument of the FT sensor
    * @param p_chain pointer to the given iDynChain
    * @param p_sensor pointer to the given iDynSensor
    * @param Y reference to the matrix where store the output matrix
    * @param excluded_link optional index (referring to the original iDynChain) of a link excluded from calculation of regressor matrix (usually because it is a virtual link introduced to describe a joint with more than one DOF)
    * @return false in case of error, true otherwise
     */
     yarp::sig::Vector iDynChainRegressorTorqueEstimation(iCub::iDyn::iDynChain *p_chain,iCub::iDyn::iDynSensor * p_sensor, const int joint_index, const int excluded_link = -1);
    
    /**
     * For a given iDynChain return a vector \f$Y_i^{\top}\f$ such that \f$\tau_i = {z_0}^{\top} {{Ad}^{\top}_{H^s_{i-1}}} W_s^s + Y_i \phi\f$ \n
     * Where phi is the vector returned by iDynChainGetBeta , \f${z_0}^{\top} = [  0 \ 0 \ 0 \ 0 \ 0 \ 1 ]\f$, \f$\tau_i\f$ is \f$i\f$ torque. \n
     * In a nutshell, this regressor represent the inertial contribution to the torque calculated through the measument of the FT sensor
    * @param p_chain pointer to the given iDynChain
    * @param p_sensor pointer to the given iDynSensor
    * @param Y reference to the matrix where store the output matrix
    * @param excluded_links vector of bool of the same length of the iDynChain, if excluded_links[i] is true the link i is excluded from the calculation of the regressor matrix (usually because it is a virtual link introduced to describe a joint with more than one DOF)
    * @return false in case of error, true otherwise
    */
     yarp::sig::Vector iDynChainRegressorTorqueEstimation(iCub::iDyn::iDynChain *p_chain,iCub::iDyn::iDynSensor * p_sensor, const int joint_index, std::vector<bool> excluded_links);
    

    /**
     * For a given iDynChain return a matrix \f$Y\f$ such that \f$ W_i^i = {Ad}^{\top}_{H^s_{i}} W_s^s + Y \phi \f$ \n
     * Where phi is the vector returned by iCub::iDyn::Regressor::iDynChainGetBeta , \f${z_0}^{\top} = [  0 \ 0 \ 0 \ 0 \ 0 \ 1 ]\f$, \f$ W_i^i \f$ is \f$i\f$ internal torque. \n
     * In a nutshell, this regressor represent the inertial contribution to the internal wrench calculated through the measument of the FT sensor
    * @param p_chain pointer to the given iDynChain
    * @param p_sensor pointer to the given iDynSensor
    * @param Y reference to the matrix where store the output matrix
    * @param excluded_link optional index (referring to the original iDynChain) of a link excluded from calculation of regressor matrix (usually because it is a virtual link introduced to describe a joint with more than one DOF)
    * @return false in case of error, true otherwise
    */
     yarp::sig::Matrix iDynChainRegressorWrenchEstimation(iCub::iDyn::iDynChain *p_chain,iCub::iDyn::iDynSensor * p_sensor, const int link_index, const int excluded_link = -1);
    
    /**
     * 
     * @param p_chain pointer to the given iDynChain
    * @param p_sensor pointer to the given iDynSensor
    * @param Y reference to the matrix where store the output matrix
    * @param excluded_links vector of bool of the same length of the iDynChain, if excluded_links[i] is true the link i is excluded from the calculation of the regressor matrix (usually because it is a virtual link introduced to describe a joint with more than one DOF)
    * @return false in case of error, true otherwise
    */
     yarp::sig::Matrix iDynChainRegressorWrenchEstimation(iCub::iDyn::iDynChain *p_chain,iCub::iDyn::iDynSensor * p_sensor, const int link_index, std::vector<bool> excluded_links);
    
    
    /**
    * For a given iDynChain calculate the regressor matrix \f$Y\f$ such that \f${W \brack \tau} = Y \phi\f$ 
    * where \f$W\f$ is the sensor wrench for the limb (the same obtained calling getSensorForceMoment),
    * \f$\tau\f$ is the vector of the torques on the joints placed in the chain after the FT sensor
    * assuming that there is no external contact on the limb
    * and phi is the vector containing the inertial parameters of the limb
    * @param p_chain pointer to the given iDynChain
    * @param p_sensor pointer to the given iDynSensor
    * @param Y reference to the matrix where store the output matrix
    * @param excluded_link optional index (referring to the original iDynChain) of a link excluded from calculation of regressor matrix (usually because it is a virtual link introduced to describe a joint with more than one DOF)
    * @return false in case of error, true otherwise
    */
     bool iDynChainRegressorComplete(iCub::iDyn::iDynChain *p_chain,iCub::iDyn::iDynSensor * p_sensor, yarp::sig::Matrix & Y, const int excluded_link = -1);
    
    
    /**
     * 
    * @param p_chain pointer to the given iDynChain
    * @param p_sensor pointer to the given iDynSensor
    * @param Y reference to the matrix where store the output matrix
    * @param excluded_links vector of bool of the same length of the iDynChain, if excluded_links[i] is true the link i is excluded from the calculation of the regressor matrix (usually because it is a virtual link introduced to describe a joint with more than one DOF)
    * @return false in case of error, true otherwise
     */
    bool iDynChainRegressorComplete(iCub::iDyn::iDynChain *p_chain,iCub::iDyn::iDynSensor * p_sensor, yarp::sig::Matrix & A, std::vector<bool> excluded_links);
    
    /**
     * For a given iDynChain return a matrix \f$Y_i\f$ such that \f$ W_i^i = Y_i \phi \f$ \n
     * Where phi is the vector returned by iCub::iDyn::Regressor::iDynChainGetBeta , \f$ W_i^i \f$ is \f$i\f$ internal torque. \n
     * @param p_chain pointer to the given iDynChain
     * @param p_sensor pointer to the given iDynSensor
     * @param Y reference to the matrix where store the output matrix
    * @param excluded_link optional index (referring to the original iDynChain) of a link excluded from calculation of regressor matrix (usually because it is a virtual link introduced to describe a joint with more than one DOF)
     * @return false in case of error, true otherwise
     */
    bool iDynChainRegressorInternalWrench(iCub::iDyn::iDynChain *p_chain,iCub::iDyn::iDynSensor * p_sensor, yarp::sig::Matrix & A, int wrench_index, const int excluded_link = -1);
    
    /**
    * @param p_chain pointer to the given iDynChain
    * @param p_sensor pointer to the given iDynSensor
    * @param Y reference to the matrix where store the output matrix
    * @param excluded_links vector of bool of the same length of the iDynChain, if excluded_links[i] is true the link i is excluded from the calculation of the regressor matrix (usually because it is a virtual link introduced to describe a joint with more than one DOF)
    * @return false in case of error, true otherwise
    */
    bool iDynChainRegressorInternalWrench(iCub::iDyn::iDynChain *p_chain,iCub::iDyn::iDynSensor * p_sensor, yarp::sig::Matrix & A, int wrench_index, std::vector<bool> excluded_links);
    

    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //iCub Limb regressors
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    /**
    * For a given limb calculate the regressor matrix \f$\Phi\f$ such that \f$W = \Phi \phi\f$ 
    * where \f$W\f$ is the sensor wrench for the limb (the same obtained calling getSensorForceMoment)
    * assuming that there is no external contact on the limb
    * and phi is the vector containing the inertial parameters of the limb
    * @param icub pointer to an iCubWholeBody object, containing the kinematic information about the robot
    * @param limbName one of right_arm,left_arm,right_leg,left_leg
    * @param Phi the regressor matrix
    * @param consider_virtual_link if true, include in the regressor calculation also the virtual link, by default false
    * @return false if some error occured, true otherwise
    */
     bool iCubLimbRegressorSensorWrench(iCub::iDyn::iCubWholeBody * icub, const std::string & limbName,yarp::sig::Matrix & Phi, bool consider_virtual_link = false);
    
    /**
    * For a given limb calculate the regressor matrix \f$\Phi\f$ such that \f${W \brack \tau} = \Phi \phi\f$ 
    * where \f$W\f$ is the sensor wrench for the limb (the same obtained calling getSensorForceMoment),
    * \f$\tau\f$ is the vector of the torques on the joints placed in the chain after the FT sensor
    * assuming that there is no external contact on the limb
    * and phi is the vector containing the inertial parameters of the limb
    * @param icub pointer to an iCubWholeBody object, containing the kinematic information about the robot
    * @param limbName one of right_arm,left_arm,right_leg,left_leg
    * @param Phi (\f$\Phi\f$) the regressor matrix
    * @param consider_virtual_link if true, include in the regressor calculation also the virtual link, by default false
    * @return false if some error occured, true otherwise
    */
     bool iCubLimbRegressorComplete(iCub::iDyn::iCubWholeBody * icub, const std::string & limbName,yarp::sig::Matrix & Phi, bool consider_virtual_link = false);
    
    /**
    * For a given limb and a given link \f$i\f$, calculate the regressor matrix \f$\Phi\f$ such that \f$W_i = \Phi \phi\f$ 
    * where \f$W_i\f$ is the internal wrench relative to joint \f$i\f$ of the limb (the same obtained calling getForce and getMoment of the iDynLink object)
    * @param icub pointer to an iCubWholeBody object, containing the kinematic information about the robot
    * @param limbName one of right_arm,left_arm,right_leg,left_leg
    * @param Phi (\f$\Phi\f$) the regressor matrix
    * @param wrench_index the index of the link relative to the internal wrench to calculate
    * @param consider_virtual_link if true, include in the regressor calculation also the virtual link, by default false
    * @return false if some error occured, true otherwise
    */
    bool iCubLimbRegressorInternalWrench(iCub::iDyn::iCubWholeBody * icub, const std::string & limbName,int wrench_index,yarp::sig::Matrix & Phi, bool consider_virtual_link = false);
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // iDynChain Get & Set Parameters Functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    /**
    * Retrive the 10 element vector of inertial parameter of the provided link of the iDynChain, in identification format
    * 
    */
    bool iDynChainGetLinkBeta(iCub::iDyn::iDynChain * p_chain,const unsigned link,yarp::sig::Vector & phi_ident);
    
    /**
    * 
    * 
    * Retrive the 10 element vector of inertial parameter of the semilink defined in iDynSensor, in identification format
    * 
    */
    bool iDynSensorGetSemilinkBeta(const iCub::iDyn::iDynSensor *p_sensor,yarp::sig::Vector & phi_ident);


    /**
    * Function used to retrieve the phi vector of the dynamical parameter of the iDynChain in identification format \n
    * The link considered are only those that come after the sensor link, plus the part of the sensor link after the sensor
    */
    bool iDynChainGetBeta(iCub::iDyn::iDynChain * p_chain, iCub::iDyn::iDynSensor * p_sensor,yarp::sig::Vector & phi_ident, std::vector<bool> excluded_links);

    /**
    * Function used to retrieve the phi vector of the dynamical parameter of the iDynChain in identification format \n
    * The link considered are only those that come after the sensor link, plus the part of the sensor link after the sensor
    */
    bool iDynChainGetBeta(iCub::iDyn::iDynChain * p_chain, iCub::iDyn::iDynSensor * p_sensor,yarp::sig::Vector & phi_ident, const int excluded_link = -1 );
    
    /** 
    * Function used to change the dynamical parameters of an iDynChain
    * @param p_chain pointer to the iCubArmNoTorsoDyn object 
    * @param p_sensor pointer to the iDynSensor object 
    * @param phi_ident vector containing the dynamical parameters of the links after the sensor, except virtual links 
    * 		   expressed in identification form (so COM and inertia matrix expressed in the link reference frame)
    * 		   the conversion to ident format is done inside this function
    */
    bool iDynChainSetBeta(iCub::iDyn::iDynChain * p_chain, iCub::iDyn::iDynSensor * p_sensor,const yarp::sig::Vector & phi_ident, std::vector<bool> excluded_links);
    

    /**
    * Function used to change the dynamical parameters of an iDynChain
    * @param p_chain pointer to the iCubArmNoTorsoDyn object 
    * @param p_sensor pointer to the iDynSensor object 
    * @param phi_ident vector containing the dynamical parameters of the links after the sensor, except virtual links 
    * 		   expressed in identification form (so COM and inertia matrix expressed in the link reference frame)
    * 		   the conversion to ident format is done inside this function
    */
    bool iDynChainSetBeta(iCub::iDyn::iDynChain * p_chain, iCub::iDyn::iDynSensor * p_sensor,const yarp::sig::Vector & phi_ident,int excluded_link = -1);
    
    
  

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // iCub Limb Get & Set Parameters Functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        
    /**
    * Set the dynamical parameters of the icub to the phi containted in the vector 
    */
    bool iCubLimbSetBeta(iCub::iDyn::iCubWholeBody * icub,std::string limbName,const yarp::sig::Vector & phi_ident,bool consider_virtual_link = false);
    
    
    /**
    * Retrieve the vector of inertial parameter for the specified limb, converting it to identification rappresentation
    * The link considered are those that are after the FT sensor. 
    * Important: the dynamical parameters of the semi-link relative to the sensor, in iDyn are expressed in a not coherent way. This function returns the "coherent" parameters.
    */
    bool iCubLimbGetBeta(const iCub::iDyn::iCubWholeBody * const icub,std::string limbName,yarp::sig::Vector & phi, bool consider_virtual_link = false);
    
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //Parameter conversion functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    
    /**
    * 
    * 
    * Function used to convert the parameter vector for one link from the rappresentation
    * used in iDyn \f$(m_i,{r_{i,C_i}^i}_x,{r_{i,C_i}^i}_y,{r_{i,C_i}^i}_z,
    * \overline{I}_xx,\overline{I}_xy,\overline{I}_xz,\overline{I}_yy,\overline{I}_yz,\overline{I}_zz)\f$
    * (where with \overline{I} is intended the Inertia with respect to the center of mass)
    * to the rappresentation used for identification, for obtaining a linear model
    * \f$(m_i,m_i{_{i,C_i}^i}_x,m_i{r_{i,C_i}^i}_y,m_i{r_{i,C_i}^i}_z,
    * \hat{I}_xx,\hat{I}_xy,\hat{I}_xz,\hat{I}_yy,\hat{I}_yz,\hat{I}_zz)\f$
    * where \hat{I} is the inertia with respect to che center of the \f$i\f$ frame.
    * 
    * @param phi_idyn the input parameters vector, expressed as are expressed in iDynLink
    * @param phi_ident the output parameters vector, expressed for identication
    */ 
    void param_conv_idyn2ident(const yarp::sig::Vector& phi_idyn, yarp::sig::Vector & phi_ident);
    
    /**
    * 
    * 
    * Function used to convert the parameter vector for one link from the rappresentation used for identification, for obtaining a linear model
    * \f$(m_i,m_i{_{i,C_i}^i}_x,m_i{r_{i,C_i}^i}_y,m_i{r_{i,C_i}^i}_z,
    * \hat{I}_xx,\hat{I}_xy,\hat{I}_xz,\hat{I}_yy,\hat{I}_yz,\hat{I}_zz)\f$
    * where \hat{I} is the inertia with respect to che center of the \f$i\f$ frame
    * to the one used in iDyn \f$(m_i,{r_{i,C_i}^i}_x,{r_{i,C_i}^i}_y,{r_{i,C_i}^i}_z,
    * \overline{I}_xx,\overline{I}_xy,\overline{I}_xz,\overline{I}_yy,\overline{I}_yz,\overline{I}_zz)\f$
    * (where with \overline{I} is intended the Inertia with respect to the center of mass)
    * 
    * @param phi_ident the input parameters vector, expressed for identication
    * @param phi_idyn the output parameters vector, expressed as are expressed in iDynLink
    */ 
    void param_conv_ident2idyn(const yarp::sig::Vector  &phi_ident,yarp::sig::Vector & phi_idyn);
    
    /**
    * Function used to convert the parameter vector for the semi link, as defined in iDyn SensorLinkNewtonEuler (so w.r.t the frame of reference of the sensor)
    * to the standard iDyn convenction, where the parameter are expressed w.r.t. the frame of reference of the link (in this case the frame of reference of the sensor link)
    * @param phi_semiLink the parameters as containted in SensorLinkNewtonEuler
    * @param H_s_i \f$H_s^i\f$ the matrix of transformation from the sensor frame of reference to the link frame of reference
    * @param phi_idyn the parameters, expressed as are expressed in OneLinkNewtonEuler
    */
    void param_conv_semiLink2idyn(const yarp::sig::Vector  &phi_semiLink, const yarp::sig::Matrix & H_s_i, yarp::sig::Vector & phi_idyn);
    
    /**
    * Function used to convert the parameter vector for the semi link from the standard iDyn convenction, where the parameter are expressed w.r.t. the frame of reference of the link (in this case the frame of reference of the sensor link)
    * to the way they are expressed in iDyn SensorLinkNewtonEuler (so w.r.t the frame of reference of the sensor)
    * @param phi_idyn the parameters vector, expressed as are expressed in OneLinkNewtonEuler
    * @param H_i_s \f$H^i_s\f$ the matrix of transformation from the sensor frame of reference to the link frame of reference
    * @param phi_semiLink the parameters as containted in SensorLinkNewtonEuler
    */
    void param_conv_idyn2semiLink(const yarp::sig::Vector  &phi_idyn,const yarp::sig::Matrix & H_i_s,yarp::sig::Vector & phi_semiLink);
    
    /**
    * Extract from a iDyn parameter vector for one link the mass 
    * @param phi_idyn the parameters vector, expressed as are expressed in iDynLink
    */
    double param_conv_idyn2mass(const yarp::sig::Vector & phi_idyn);
    
    /**
    * Extract from a iDyn parameter vector for one link the center of mass vector
    * @param phi_idyn the parameters vector, expressed as are expressed in iDynLink
    */
    yarp::sig::Vector param_conv_idyn2com(const yarp::sig::Vector & phi_idyn);
    
    /**
    * Extract from a iDyn parameter vector for one link the center of mass matrix
    * @param phi_idyn the parameters vector, expressed as are expressed in iDynLink
    */
    yarp::sig::Matrix param_conv_idyn2COM(const yarp::sig::Vector & phi_idyn);
    
    /**
    * Extract from a iDyn parameter vector for one link the inertia matrix
    * @param phi_idyn the parameters vector, expressed as are expressed in iDynLink
    * @return the 3x3 inertia matrix
    */
    yarp::sig::Matrix param_conv_idyn2inertia(const yarp::sig::Vector & phi_idyn);
    
        
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //Misc functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    /**
    * 
    * 
    * Compute the matrix \f$\mathbf{M(\omega,\dot{\omega})}\f$ such that
    * \f$\mathbf{I_{mat} \dot{\omega}} + \omega \times \mathbf{I}_{mat} \omega =
    * \mathbf{M(\omega,\dot{\omega})} \mathbf{I}_{vet}\f$
    * Where \f$\mathbf{I}_{mat}\f$ is the inertia (symmetric) matrix and \f$\mathbf{I}_{vet}\f$ is the corrisponding 6 element vector. 
    */
     yarp::sig::Matrix EulerEquationsRegressor(const yarp::sig::Vector & w,const yarp::sig::Vector& dw);
    
    /**
    * Calculate the \f$H_{index}^s\f$ matrix, or \f$H_s^{index}\f$ if inv == true
    */
     yarp::sig::Matrix H_wrt_sensor(iCub::iDyn::iDynChain * p_chain, iCub::iDyn::iDynSensor * p_sensor, const unsigned int index, bool inv = false);
    
    /**
    * 
    * 
    * Calculate the \f$H^a_{b}\f$ matrix, the transformation matrix from the reference frame of link \f$b\f$
    * to the reference frame of link \f$a\f$
    */
     yarp::sig::Matrix H_b_wrt_a(iCub::iDyn::iDynChain * p_chain,  int a_index, int b_index);
    
    /**
    * 
    * 
    * Take a vector of bools, and set only one element, and set to false the rest of the vector
    * @param bool_vec vector of bools to modify
    * @param index index of the element to set
    * @return true if all went right, false otherwise
    * 
    */
     bool setOnlyOneElement(std::vector<bool> & bool_vec, const int index);

    
    /**
    * Convert a 3x3 simmetric Matrix \f$A\f$ in a 6 element Vector \f$[ A_{00} \ A_{01} \ A_{02} \ A_{11} \ A_{12} \ A_{22} ]\f$ 
    * @param symMat a reference to the input 3x3 symmetric Matrix
    * @return the Vector containg the elements of the symmetric Matrix
    */
    yarp::sig::Vector symmMatrix2Vector(const yarp::sig::Matrix& symMat);
    
    /**
    * 
    * 
    * Convert an 6 element Vector in a 3x3 symmetric Matrix, using the same convention of symmMatrix2Vector
    * @param ret a reference to the 6 element input Vector
    * @return the 6x6 symmetric Matrix
    */ 
    yarp::sig::Matrix Vector2symmMatrix(const yarp::sig::Vector & ret);
    
    /**
    * Extract useful data from the icub object passed, function for internal use
    * @param icub pointer to iCubWholeBody input object
    * @param limb name of the limb for which information is returned
    * @param consider_virtual_link if true also the virtual link is considered, otherwise not
    * @param p_chain pointer to the iDynChain of the selected limb
    * @param p_sensor pointer to the iDynSensor of the selected limb
    * @return false in case of error, true otherwise
    */ 
    bool iCubLimbGetData(const iCub::iDyn::iCubWholeBody * const icub, const std::string & limbName,const bool consider_virtual_link, iCub::iDyn::iDynChain * & p_chain, iCub::iDyn::iDynSensor * & p_sensor,int & virtual_link );
    
}
    
}
    
}
    
    
#endif
    
