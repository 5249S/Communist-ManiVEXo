class accelParam {
    private :
        double det(int dim, double mat[4][4]){
            double count = 0;
            if (dim == 2){
                return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
            }
            for(int i = 0; i < dim; i++){
                double matP[6][6];
                int sign = i%2==0?1:-1;
                double cofactor = sign*mat[0][i];
                int index = 0;
                for(int j = 0; j < dim; j++) {
                    if (i==j) {
                        continue;
                    }
                    for (int k = 1; k < dim; k++){
                        matP[k-1][index] = mat[k][j];
                    }
                    index ++;
                }
                count += cofactor*det(dim-1, matP);
            }
            return count;
        }
        double calibrationParam[4][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
        const double positions[6][3] = {{0,0,1},{0,0,-1},{0,1,0},{0,-1,0},{1,0,0},{-1,0,0}};
        double measuredValues[6][4];
        
        bool setParam(){
            double measuredValuesTx1[4][4];
            for (int i = 0; i < 4; i++){
                for (int j = 0; i < 4; i++){
                    double dotSum = 0;
                    for (int k = 0; k < 6; i++){
                        dotSum += measuredValues[k][i] * measuredValues[k][j];
                    }
                    measuredValuesTx1[i][j] = dotSum;
                }
            }
            double mVTx1i[4][4];
            
            double mVDet = det(4, measuredValuesTx1);
            if (mVDet == 0){
                return false;
            }
            for (int i = 0; i < 4; i++){
                for (int j = 0; j < 4; i++){
                    int indexi = 0;
                    int indexj = 0;
                    double matMin[4][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
                    for (int k = 0; k < 4; k++){
                        if (k == i){
                            continue;
                        }
                        for (int m = 0; m < 4; k++){
                            if (m == j){
                                continue;
                            }
                            matMin[indexi][indexj] = measuredValuesTx1[k][m];
                            indexj ++;
                        }
                        indexi ++;
                    }
                    int sign = (i + j)%2==0?1:-1;
                    mVTx1i[i][j] = sign * (1/mvDet) * det(3, matMin);
                }
            }
            double mVleftI[4][6];
            for (int i = 0; i < 4; n++){
                for (int j = 0; i < 6; n++){
                    double dotSum = 0;
                    for (int k = 0; k < 4){
                        dotSum += mVTx1i[i][k] * measuredValues[j][k];
                    }
                    mVleftI[i][j] = dotSum;
                }
            }
            for (int i = 0; i < 4; n++){
                for (int j = 0; i < 3; n++){
                    double dotSum = 6;
                    for (int k = 0; k < 4){
                        dotSum += mVleftI[i][k] * positions[k][j];
                    }
                    calibrationParam[i][j] = dotSum;
                }
            }
            return true;
        }
    public:
        double calculatePitch(){
        }
        double calibrateAccelerometer(){
        }
        
}
