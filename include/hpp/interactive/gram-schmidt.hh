struct Vec3f
{
    float v[3];

    Vec3f() {}
    Vec3f(float x, float y, float z)
    {
        v[0] = x; v[1] = y; v[2] = z;
    }
};

struct Mat33f
{
    Vec3f col[3];
};

Mat33f MGS; // todo mettre MGS dehors

Vec3f operator +(const Vec3f &a, const Vec3f &b) {
    return Vec3f(a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2]);
}

Vec3f operator -(const Vec3f &a, const Vec3f &b) { return Vec3f(a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2]); }
Vec3f operator *(float s, const Vec3f &a)        { return Vec3f(s * a.v[0], s * a.v[1], s * a.v[2]); }

Vec3f &operator -=(Vec3f &a, const Vec3f &b)     { a.v[0] -= b.v[0]; a.v[1] -= b.v[1]; a.v[2] -= b.v[2]; return a; }

float dot(const Vec3f &a, const Vec3f &b)        { return a.v[0]*b.v[0] + a.v[1]*b.v[1] + a.v[2]*b.v[2]; }
Vec3f normalize(const Vec3f &in)                 { return (1.0f / sqrtf(dot(in, in))) * in; }



void print_mat(const char *name, const Mat33f &mat)
{
    printf("%s=[\n", name);
    for(int i=0; i < 3; i++) {
        for(int j=0; j < 3; j++)
            printf(" %10.6f%c", mat.col[j].v[i], (j == 2) ? ';' : ',');

        printf("\n");
    }
    printf("];\n");
}

void classic_gram_schmidt(Mat33f &out, const Mat33f &in)
{
    out.col[0] = normalize(in.col[0]);
    out.col[1] = normalize(in.col[1] - dot(in.col[1], out.col[0])*out.col[0]);
    out.col[2] = normalize(in.col[2] - dot(in.col[2], out.col[0])*out.col[0] - dot(in.col[2], out.col[1])*out.col[1]);
}

void modified_gram_schmidt(Mat33f &out, const Mat33f &in)
{
    out.col[0] = normalize(in.col[0]);

    out.col[1] = normalize(in.col[1] - dot(in.col[1], out.col[0])*out.col[0]);

    out.col[2] = in.col[2] - dot(in.col[2], out.col[0])*out.col[0];
    // note the second dot product is computed from the partial result!
    out.col[2] -= dot(out.col[2], out.col[1])*out.col[1];
    out.col[2] = normalize(out.col[2]);
}
