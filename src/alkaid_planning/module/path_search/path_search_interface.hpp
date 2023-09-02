namespace module_ns
{
class PathSearchInterface
{
    PathSearchInterface() = default;
    virtual void MapBuild() = 0;
    virtual void MapUpdate() = 0;
    virtual bool SetStartPoint() = 0;
    virtual bool SetEndPoint() = 0;
    virtual bool GetSearchPath() = 0;
    virtual ~PathSearchInterface()=default;

}; // class PathSearchInterface
} // namespace module_ns
