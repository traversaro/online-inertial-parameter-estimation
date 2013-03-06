#include "MatVetIO.h"
#include <cstdio>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <stdint.h>
#include <cassert>
#include <iostream>

bool Vector_write(const std::string file_name, const yarp::sig::Vector & vec)
{
    FILE * fp;
    uint32_t len;
    double elem;
    
    len = vec.size();
    
    fp = fopen(file_name.c_str(),"wb");
    if( fp == NULL ) return false;
    
    fwrite(&len,sizeof(uint32_t),1,fp);
    
    for(size_t i=0;i<len;i++) { 
        elem = vec[i];
        if(1 != fwrite(&elem,sizeof(double),1,fp)) {
            assert(false);
        }
    }
    
    /*
    if( gsl_vector_fwrite(fp,(gsl_vector *)vec.getGslVector()) == GSL_EFAILED ) {
        fclose(fp);
        return false;
    }*/
    
    fclose(fp);
    return true;
}

bool Vector_read(const std::string file_name, yarp::sig::Vector & vec)
{
    FILE * fp;
    uint32_t len;
    double elem;
    
    std::cerr << "Opening vector " << file_name << std::endl;

    
    fp = fopen(file_name.c_str(),"rb");
    if( fp == NULL ) {
        std::cerr << "Error opening vector " << file_name << std::endl;
        return false;
    }
    
    fread(&len,sizeof(uint32_t),1,fp);
    vec.resize(len);
    
    for(size_t i=0;i<len;i++) { 
        fread(&elem,sizeof(double),1,fp);
        vec[i]=elem;
    }
    
    /*
    if( gsl_vector_fread(fp,(gsl_vector*)vec.getGslVector()) == GSL_EFAILED ) {
        fclose(fp);
        return false;
    }*/
    fclose(fp);
    return true;
}

bool Matrix_write(const std::string file_name, const yarp::sig::Matrix & mat)
{
    FILE * fp;
    uint32_t rows,cols;
    double elem;
    
    rows = mat.rows();
    cols = mat.cols();
    
    fp = fopen(file_name.c_str(),"wb");
    if( fp == NULL ) return false;
    
    //writing dimensions informations
    fwrite(&rows,sizeof(uint32_t),1,fp);
    fwrite(&cols,sizeof(uint32_t),1,fp);

    for(size_t i=0;i<rows;i++) {
        for(size_t j=0;j<cols;j++ ) {
            elem = mat(i,j);
            if(fwrite(&elem,sizeof(double),1,fp) != 1) {
                assert(false);
            };
         }
     }
    /*
    if( gsl_matrix_fwrite(fp,(gsl_matrix*)mat.getGslMatrix()) == GSL_EFAILED ) {
        fclose(fp);
        return false;
    }*/
    fclose(fp);
    return true;
}

bool Matrix_read(const std::string file_name, yarp::sig::Matrix & mat)
{
    FILE * fp;
    uint32_t rows,cols;
    double elem;
    
    std::cerr << "Opening matrix " << file_name << std::endl;

    
    fp = fopen(file_name.c_str(),"rb");
    if( fp == NULL ) {
        std::cerr << "Error opening matrix " << file_name << std::endl;
        return false;
    }
    
    //reading dimensions information
    fread(&rows,sizeof(uint32_t),1,fp);
    fread(&cols,sizeof(uint32_t),1,fp);
    
    mat.resize(rows,cols);
    
    for(size_t i=0;i<rows;i++) {
        for(size_t j=0;j<cols;j++ ) {
             if(fread(&elem,sizeof(double),1,fp) != 1) {
                 assert(false);
             }
             mat(i,j) = elem;
         }
     }
    
    /*
    if( gsl_matrix_fread(fp,(gsl_matrix*)mat.getGslMatrix()) == GSL_EFAILED ) {
        fclose(fp);
        return false;
    }
    */
    
    
    fclose(fp);
    return true;
}
