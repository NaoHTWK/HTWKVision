#include "ellifit.h"

#include <algorithm>

using namespace std;

namespace htwk {

static float S[7][7];
static float L[7][7];
static float C[7];

static float invL[7][7];

void elli_init() {
    for(int j=0;j<7;j++){
        for(int i=0;i<7;i++){
            S[j][i]=0;
            L[j][i]=0;
            invL[j][i]=0;
        }
    }
}

bool fit(vector<point_2d> points,float *result) {
    auto newEndIt = unique(points.begin(), points.end());
    points.erase(newEndIt, points.end());

    int np = points.size();
    if (np < 6)
        return false;

    float S11=0,S12=0,S13=0,S14=0,S15=0,S16=0,S23=0,S25=0,S26=0,S33=0,S35=0,S36=0,S46=0,S56=0;
    for (int i = 0; i < np; i++) {
        point_2d p=points[i];
        float tx = p.x;
        float ty = p.y;
        float txtx=tx*tx;
        float txty=tx*ty;
        float tyty=ty*ty;
        S11+=txtx*txtx;
        S12+=txtx*txty;
        S13+=txtx*tyty;
        S23+=txty*tyty;
        S33+=tyty*tyty;
        S14+=txtx*tx;
        S15+=txtx*ty;
        S25+=txty*ty;
        S35+=tyty*ty;
        S16+=txtx;
        S26+=txty;
        S36+=tyty;
        S46+=tx;
        S56+=ty;
    }
    S[1][1]=S11;
    S[2][1]=S[1][2]=S12;
    S[2][2]=S[3][1]=S[1][3]=S13;
    S[3][2]=S[2][3]=S23;
    S[3][3]=S33;
    S[4][1]=S[1][4]=S14;
    S[4][2]=S[2][4]=S[5][1]=S[1][5]=S15;
    S[4][3]=S[3][4]=S[5][2]=S[2][5]=S25;
    S[5][3]=S[3][5]=S35;
    S[4][4]=S[6][1]=S[1][6]=S16;
    S[5][4]=S[4][5]=S[6][2]=S[2][6]=S26;
    S[5][5]=S[6][3]=S[3][6]=S36;
    S[6][4]=S[4][6]=S46;
    S[6][5]=S[5][6]=S56;
    S[6][6]=np;

    if(choldc(S, 6, L)!=0)
        return false;


    if(inverse(L, invL, 6) != 0)
        return false;

    C[1] = invL[1][1] * -4 * invL[1][3] + invL[1][2] * invL[1][2];
    C[2] = invL[2][1] * -4 * invL[2][3] + invL[2][2] * invL[2][2];
    C[3] = invL[3][1] * -4 * invL[3][3] + invL[3][2] * invL[3][2];
    C[4] = invL[4][1] * -4 * invL[4][3] + invL[4][2] * invL[4][2];
    C[5] = invL[5][1] * -4 * invL[5][3] + invL[5][2] * invL[5][2];
    C[6] = invL[6][1] * -4 * invL[6][3] + invL[6][2] * invL[6][2];

    for (int j=1;j<=6;j++)  /* Scan columns */
      {
        float mod = 0.0;
        for (int i=1;i<=6;i++)
          mod += invL[j][i]*invL[j][i];

        if(mod == 0)
            return false;

        for (int i=1;i<=6;i++)
            invL[j][i] /=  sqrtf(mod);
      }

    float zero = 10e-20;
    int solind = 0;

    for (int j = 1; j <= 6; j++)
        if (C[j] < -zero)
            solind = j;

    // Now fetch the right solution
    bool allZero=true;
    for (int i = 1; i <= 6; i++) {
        result[i-1] = invL[solind][i];
        if(result[i-1]!=0)
            allZero=false;
    }
    return !allZero;
}

// Perform the Cholesky decomposition
// Return the lower triangular L  such that L*L'=A
int choldc(float a[][7], int n, float l[][7]) {
    int i, j, k;
    float sum;
    float *p = (float*)malloc(sizeof(float)*(n + 1));
    if (p == nullptr) {
        fprintf(stderr, "Error: malloc() returned NULL in file %s line %d. Exiting.\n", __FILE__, __LINE__);
        exit(EXIT_FAILURE);
    }

    for (i = 1; i <= n; i++) {
        for (j = i; j <= n; j++) {
            for (sum = a[i][j], k = i - 1; k >= 1; k--)
                sum -= a[i][k] * a[j][k];
            if (i == j) {
                if (sum <= 0.0f)
                {
                    free(p);
                    return -1;
                } else
                    p[i] = sqrtf(sum);
            } else {
                if(p[i] == 0) {
                    free(p);
                    return -1;
                }
                a[j][i] = sum / p[i];
            }
        }
    }
    for (i = 1; i <= n; i++)
        for (j = i; j <= n; j++)
            if (i == j)
                l[i][i] = p[i];
            else {
                l[j][i] = a[j][i];
                l[i][j] = 0.0f;
            }
    free(p);
    return 0;
}

int inverse(float TB[][7], float InvB[][7], int N) {
    int k, i, j, p, q;
    float mult;
    float D, temp;
    float maxpivot;
    int npivot;
    float A[N+1][2*N+2];
    float eps = 10e-20;

    for (k = 1; k <= N; k++) {
        for (j = 1; j <= N; j++)
            A[k][j] = TB[k][j];
        for (j = N + 1; j <= 2 * N + 1; j++)
            A[k][j] = (float) 0;
        A[k][k - 1 + N + 2] = (float) 1;
    }
    for (k = 1; k <= N; k++) {
        maxpivot = fabs((float) A[k][k]);
        npivot = k;
        for (i = k; i <= N; i++)
            if (maxpivot < fabs((float) A[i][k])) {
                maxpivot = fabs((float) A[i][k]);
                npivot = i;
            }
        if (maxpivot >= eps) {
            if (npivot != k)
                for (j = k; j <= 2 * N + 1; j++) {
                    temp = A[npivot][j];
                    A[npivot][j] = A[k][j];
                    A[k][j] = temp;
                };
            D = A[k][k];
            if(D==0 || D == numeric_limits<float>::infinity() || D == -numeric_limits<float>::infinity())
                return -1;
            for (j = 2 * N + 1; j >= k; j--)
                A[k][j] = A[k][j] / D;
            for (i = 1; i <= N; i++) {
                if (i != k) {
                    mult = A[i][k];
                    for (j = 2 * N + 1; j >= k; j--)
                        A[i][j] = A[i][j] - mult * A[k][j];
                }
            }
        } else { // The matrix may be singular
            return (-1);
        };
    }
    for (k = 1, p = 1; k <= N; k++, p++)
        for (j = N + 2, q = 1; j <= 2 * N + 1; j++, q++)
            InvB[p][q] = A[k][j];
    return (0);
} /*  End of INVERSE   */

}  // namespace htwk
