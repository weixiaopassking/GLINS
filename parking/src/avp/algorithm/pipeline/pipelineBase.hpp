#ifndef _PIPELINE_BASE_HPP
#define _PIPELINE_BASE_HPP

class pipelineBase
{
public:
    virtual bool init() = 0;
    virtual void run() = 0;
    virtual void save() = 0;
};

#endif