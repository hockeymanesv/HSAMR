package tileview.demo;

import android.os.Bundle;
import android.view.View;
import android.widget.ImageView;

import com.qozix.tileview.TileView;
import com.qozix.tileview.markers.MarkerLayout;

import java.util.ArrayList;

public class RealMapTileViewActivity extends TileViewActivity {

  public static final double NORTH_WEST_LATITUDE = 39.9639998777094;
  public static final double NORTH_WEST_LONGITUDE = -75.17261900652977;
  public static final double SOUTH_EAST_LATITUDE = 39.93699709962642;
  public static final double SOUTH_EAST_LONGITUDE = -75.12462846235614;

  @Override
  public void onCreate( Bundle savedInstanceState ) {

    super.onCreate( savedInstanceState );

    // we'll reference the TileView multiple times
    TileView tileView = getTileView();

    // size and geolocation
    tileView.setSize( 8967, 6726 );

    // we won't use a downsample here, so color it similarly to tiles
    tileView.setBackgroundColor( 0xFFe7e7e7 );

    tileView.addDetailLevel( 0.0125f, "tiles/map/phi-62500-%d_%d.jpg" );
    tileView.addDetailLevel( 0.2500f, "tiles/map/phi-125000-%d_%d.jpg" );
    tileView.addDetailLevel( 0.5000f, "tiles/map/phi-250000-%d_%d.jpg" );
    tileView.addDetailLevel( 1.0000f, "tiles/map/phi-500000-%d_%d.jpg" );

    // markers should align to the coordinate along the horizontal center and vertical bottom
    tileView.setMarkerAnchorPoints( -0.5f, -1.0f );

    // provide the corner coordinates for relative positioning
    tileView.defineBounds(
      NORTH_WEST_LONGITUDE,
      NORTH_WEST_LATITUDE,
      SOUTH_EAST_LONGITUDE,
      SOUTH_EAST_LATITUDE
    );

    // add markers for all the points
    for( double[] point : points ) {
      // any view will do...
      ImageView marker = new ImageView( this );
      // save the coordinate for centering and callout positioning
      marker.setTag( point );
      // give it a standard marker icon - this indicator points down and is centered, so we'll use appropriate anchors
      marker.setImageResource( Math.random() < 0.75 ? R.drawable.map_marker_normal : R.drawable.map_marker_featured );
      // on tap show further information about the area indicated
      // this could be done using a OnClickListener, which is a little more "snappy", since
      // MarkerTapListener uses GestureDetector.onSingleTapConfirmed, which has a delay of 300ms to
      // confirm it's not the start of a double-tap. But this would consume the touch event and
      // interrupt dragging
      tileView.getMarkerLayout().setMarkerTapListener( markerTapListener );
      // add it to the view tree
      tileView.addMarker( marker, point[0], point[1], null, null );
    }

    // let's start off framed to the center of all points
    double x = 0;
    double y = 0;
    for( double[] point : points ) {
      x = x + point[0];
      y = y + point[1];
    }
    int size = points.size();
    x = x / size;
    y = y / size;
    frameTo( x, y );

    // test higher than 1
    tileView.setScaleLimits( 0, 2 );

    // start small and allow zoom
    tileView.setScale( 0.5f );

    // with padding, we might be fast enough to create the illusion of a seamless image
    tileView.setViewportPadding( 256 );

    // we're running from assets, should be fairly fast decodes, go ahead and render asap
    tileView.setShouldRenderWhilePanning( true );

  }

  private MarkerLayout.MarkerTapListener markerTapListener = new MarkerLayout.MarkerTapListener() {

    @Override
    public void onMarkerTap( View view, int x, int y ) {
      // get reference to the TileView
      TileView tileView = getTileView();
      // we saved the coordinate in the marker's tag
      double[] position = (double[]) view.getTag();
      // lets center the screen to that coordinate
      tileView.slideToAndCenter( position[0], position[1] );
      // create a simple callout
      SampleCallout callout = new SampleCallout( view.getContext() );
      // add it to the view tree at the same position and offset as the marker that invoked it
      tileView.addCallout( callout, position[0], position[1], -0.5f, -1.0f );
      // a little sugar
      callout.transitionIn();
      // stub out some text
      callout.setTitle( "MAP CALLOUT" );
      callout.setSubtitle( "Info window at coordinate:\n" + position[1] + ", " + position[0] );
    }
  };

  // a list of points to demonstrate markers and paths
  private ArrayList<double[]> points = new ArrayList<>();

  {
    points.add( new double[] {-75.1489070, 39.9484760} );
    points.add( new double[] {-75.1494000, 39.9487722} );
    points.add( new double[] {-75.1468350, 39.9474180} );
    points.add( new double[] {-75.1472000, 39.9482000} );
    points.add( new double[] {-75.1437980, 39.9508290} );
    points.add( new double[] {-75.1479650, 39.9523130} );
    points.add( new double[] {-75.1445500, 39.9472960} );
    points.add( new double[] {-75.1506100, 39.9490630} );
    points.add( new double[] {-75.1521278, 39.9508083} );
    points.add( new double[] {-75.1477600, 39.9475320} );
    points.add( new double[] {-75.1503800, 39.9489900} );
    points.add( new double[] {-75.1464200, 39.9482000} );
    points.add( new double[] {-75.1464850, 39.9498500} );
    points.add( new double[] {-75.1487030, 39.9524300} );
    points.add( new double[] {-75.1500167, 39.9488750} );
    points.add( new double[] {-75.1458360, 39.9479700} );
    points.add( new double[] {-75.1498222, 39.9515389} );
    points.add( new double[] {-75.1501990, 39.9498900} );
    points.add( new double[] {-75.1460060, 39.9474210} );
    points.add( new double[] {-75.1490230, 39.9533960} );
    points.add( new double[] {-75.1471980, 39.9485350} );
    points.add( new double[] {-75.1493500, 39.9490200} );
    points.add( new double[] {-75.1500910, 39.9503850} );
    points.add( new double[] {-75.1483930, 39.9485040} );
    points.add( new double[] {-75.1517260, 39.9473720} );
    points.add( new double[] {-75.1525630, 39.9471360} );
    points.add( new double[] {-75.1438400, 39.9473390} );
    points.add( new double[] {-75.1468240, 39.9495400} );
    points.add( new double[] {-75.1466410, 39.9499900} );
    points.add( new double[] {-75.1465050, 39.9501110} );
    points.add( new double[] {-75.1473460, 39.9436200} );
    points.add( new double[] {-75.1501570, 39.9480430} );
  }
}