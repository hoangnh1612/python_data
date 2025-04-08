<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface address="192.168.44.122" multicast="true"/> ### our ip
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer Address="192.168.44.124"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>

export CYCLONEDDS_URI=file:///home/.../cyclonedds_config.xml

export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
