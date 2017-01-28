/* WARNING this method has to be called before sampling noise. */
void init (void);
double simplex_noise(double x, double y, double z, int octaves, double persistence);
