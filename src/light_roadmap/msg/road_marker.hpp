namespace msg
{

class RoadMarker
{
  public:
    double confidence;
};

class LaneLine : public RoadMarker
{
  public:
};

class StopLine : public RoadMarker
{
  public:
};

class TrafficMarker : public RoadMarker
{
  public:
};
} // namespace msg
