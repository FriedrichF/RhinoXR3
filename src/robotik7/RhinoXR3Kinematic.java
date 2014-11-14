package robotik7;

import java.util.Vector;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.atan2;
import static java.lang.Math.acos;
import static java.lang.Math.PI;
import static java.lang.Math.E;
import static java.lang.Math.log;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class RhinoXR3Kinematic {
	//Der überschaubarkeit zu gute, wurde weniger kaskadiert und einige deklarationen mehrfach ausgeführt
	
	
	private final Double a2 = 0.2286;
	private final Double a3 = 0.2286;
	private final Double a4 = 0.095;
	
	private final Double d1 = 0.2604;
	private final Double d5 = 0.1683;
	
	private Double q234;
	private Double b1;
	private Double b2;
	
	
	public Vector<Double> getGeschwindigkeitGelenkvariabel(Vector<Double> w, Vector<Double> wt, Vector<Double> q){
		
		//t für Ableitung
		Double qt1, qt2, qt3, qt4, qt5;
		qt1 = bestimme_qt1(w.get(0), wt.get(1), w.get(1), wt.get(0));
		
		return null;
	}
	
	
	private Double bestimme_qt1(Double w1, Double wt2, Double w2, Double wt1 ){
		
		//skript 4 seite 51, Geschwindigkeit basis
		Double qt1 = (w1*wt2-w2*w1)/(w1*w1+w2*w2);
		
		return qt1;
		
	}
	
	public Vector<Double> getToolConfigVektor(Vector<Double> gelenkvariablen_vektor){
		
		if(gelenkvariablen_vektor.size() != 5){
			throw new IllegalArgumentException("gelenkvariablen_vektor hat die falsche länge!!!");
		}
		
		Double q1, q2, q3, q4, q5;
		q1	= gelenkvariablen_vektor.get(0);
		q2	= gelenkvariablen_vektor.get(1);		
		q3	= gelenkvariablen_vektor.get(2);
		q4	= gelenkvariablen_vektor.get(3);
		q5	= gelenkvariablen_vektor.get(4);
		
		Double w1,w2,w3,w4,w5,w6;
		w1	= bestimme_w1(q1, q2, q3, q4);
		w2	= bestimme_w2(q1, q2, q3, q4);		
		w3	= bestimme_w3(q1, q2, q3, q4);		
		w4	= bestimme_w4(q1, q2, q3, q4, q5);
		w5	= bestimme_w5(q1, q2, q3, q4, q5);
		w6	= bestimme_w6(q1, q2, q3, q4, q5);
		
		Vector<Double> w = new Vector<Double>();
		w.add(w1);
		w.add(w2);
		w.add(w3);
		w.add(w4);
		w.add(w5);
		w.add(w6);
		
		return w;
	}
	
	
	
	private Double bestimme_w1(Double q1, Double q2, Double q3, Double q4){

		q234		= q2+q3+q4;
		Double q23	= q2+q3;
		
		//w1 = C1(a2C2+a3C23+a4C234-d5S234)
		Double w1 	= cos(q1)*(a2*cos(q2)+a3*cos(q23)+a4*cos(q234)-d5*sin(q234));
		
		return w1;
		
	}
	
	private Double bestimme_w2(Double q1, Double q2, Double q3, Double q4){

		q234		= q2+q3+q4;
		Double q23	= q2+q3;
		
		//w2 = S1(a2C2+a3C23+a4C234-d5S234)
		Double w2 	= sin(q1)*(a2*cos(q2)+a3*cos(q23)+a4*cos(q234)-d5*sin(q234));
		
		return w2;
		
	}
	
	private Double bestimme_w3(Double q1, Double q2, Double q3, Double q4){

		q234		= q2+q3+q4;
		Double q23	= q2+q3;
		
		//w3 = d1-a2S-a3S2-a4S234-d5C234
		Double w3 	= d1	-a2*sin(q2)	-a3*sin(q23) 	-a4*sin(q234)	-d5*cos(q234);
		
		return w3;
		
	}
	
	private Double bestimme_w4(Double q1, Double q2, Double q3, Double q4, Double q5){

		q234		= q2+q3+q4;
		
		//w4 = -exp(q5/pi)C1*S234
		
		Double w4 	= -pow(E, (q5/PI))*cos(q1)*sin(q234);
		
		return w4;
		
	}
	
	private Double bestimme_w5(Double q1, Double q2, Double q3, Double q4, Double q5){

		q234		= q2+q3+q4;
		
		//w4 = -exp(q5/pi)S1*S234
		
		Double w5 	= -pow(E, (q5/PI))*sin(q1)*sin(q234);
		
		return w5;
		
	}
	
	private Double bestimme_w6(Double q1, Double q2, Double q3, Double q4, Double q5){

		q234		= q2+q3+q4;
		
		//w4 = -exp(q5/pi)C234
		
		Double w6 	= -pow(E, (q5/PI))*cos(q234);
		
		return w6;
		
	}
	
	public Vector<Double> getGelenkvariablenVektor(Vector<Double> tool_config_vektor){
		
		Vector<Double> w = tool_config_vektor;
		
		if(w.size() != 6){
			throw new IllegalArgumentException("Tool config Vektor hat die falsche länge!!!");
		}
		
		Double q1,q2,q3,q4,q5;
		Double w1, w2;
		
		w1	= w.elementAt(0);	
		w2	= w.elementAt(1);
		
		q1 	= bestimme_q1(w1, w2);
		q3	= bestimme_q3(w, q1);
		q2	= bestimme_q2(q3);
		q4	= bestimme_q4(q2, q3);
		q5	= bestimme_q5(tool_config_vektor);
		
		Vector<Double> gelenkvariablen_vektor = new Vector<Double>();
		gelenkvariablen_vektor.add(q1);
		gelenkvariablen_vektor.add(q2);
		gelenkvariablen_vektor.add(q3);
		gelenkvariablen_vektor.add(q4);
		gelenkvariablen_vektor.add(q5);

		
		
		return gelenkvariablen_vektor;
	}
	
	private Double bestimme_q1(Double w1, Double w2){
		return atan2(w2, w1);
	}
	
	private Double bestimme_q3(Vector<Double> tool_config_vektor, Double q1){

		Double w1	= tool_config_vektor.get(0);
		Double w2	= tool_config_vektor.get(1);
		Double w3	= tool_config_vektor.get(2);
		Double w4	= tool_config_vektor.get(3);
		Double w5	= tool_config_vektor.get(4);
		Double w6	= tool_config_vektor.get(5);
		

		
		this.q234 = atan2(-(cos(q1)*w4	+sin(q1)*w5),		-w6);
		System.out.println(q234+" q234");
		this.b1	= cos(q1)*w1	+sin(q1)*w2		-a4*cos(q234)	+ d5*sin(q234);
		System.out.println(b1+" b1");
		this.b2	= d1	-a4*sin(q234)	-d5*cos(q234)	-w3;
		System.out.println(b2+" b2");
		System.out.println((b1*b1+b2*b2-a2*a2-a3*a3)/(2*a2*a3)+ "  acos");
		System.out.println(acos(((b1*b1)+(b2*b2)-(a2*a2)-(a3*a3))/(2*a2*a3))+ "  acos");
		Double q3	= acos(((b1*b1)+(b2*b2)-(a2*a2)-(a3*a3))/(2*a2*a3));
		
		return q3;
	}
	
	private Double bestimme_q2(Double q3){
		
		Double x 	= (a2+a3*cos(q3))*b2-a3*sin(q3)*b1;
		Double y 	= (a2+a3*cos(q3))*b1+a3*sin(q3)*b2;
		
		Double q2 	= atan2(x, y);
		
		return q2;
	}
	
	private Double bestimme_q4(Double q2, Double q3){
		
		Double q4 = q234-q2-q3;
		
		return q4;
	}
	
	private Double bestimme_q5(Vector<Double> tool_config_vektor){
		
		Double w4 = tool_config_vektor.get(3);
		Double w5 = tool_config_vektor.get(4);
		Double w6 = tool_config_vektor.get(5);
		
		Double q5 = PI*log(sqrt(w4*w4+w5*w5+w6*w6));
		
		return q5;
	}
	
	
	public String vectorToString(Vector<Double> v){
		
		String s="";
		
		for(Double d: v){
			s+=d+"\r";
		}
		
		return s+"\r";
		
	}
}
