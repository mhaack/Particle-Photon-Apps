/***************************************************************************
  A simple test to expose the app name (based on filename) and date & time
  of compilation.
 ***************************************************************************/

void setup()
{
    Particle.variable("appname", strrchr((const char*)__FILE__, '/')+1, STRING);
    Particle.variable("version",  (const char*)(__DATE__ " " __TIME__), STRING);
}
//
void loop()
{

}
