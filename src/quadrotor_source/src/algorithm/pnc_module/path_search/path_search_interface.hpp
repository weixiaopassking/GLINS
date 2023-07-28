#ifndef _PATH_SEARCH_INTERFACE
#define _PATH_SEARCH_INTERFACE

class PathSearchInterface
{
    PathSearchInterface()
    {
    }
    virtual void SetMap() = 0;
    virtual void SetSatrtPoint() = 0;
    virtual void SetEndPoint() = 0;
    virtual  ~PathSearchInterface()
    {
    }
};

#endif //_PATH_SEARCH_INTERFACE