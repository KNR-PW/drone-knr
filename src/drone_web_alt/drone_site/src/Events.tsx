import { Component, For, createResource } from "solid-js";


type Detection = {
  detection_id: number,
  date: Date, // change to date??
  lat: number,
  lon: number, 
  photo: string,
  trees: Tree[],
}

type Tree = {
  tree_id: number,
  color: string,
}

const URL = 'http://localhost:5000';
const fetchData = async () => {
  const resp = await fetch(URL);
  const json = await resp.json();
  const detections: Detection[] = json.map((o: any) => ({ ...o, date: new Date(o.date) }));

  detections.sort((a, b) => b.date.getTime() - a.date.getTime());
  
  return detections;
};


interface EventProps {
  detection: Detection
}

const Event = (props: EventProps) =>
  <div><p>{props.detection.detection_id}</p><p>{props.detection.date + ''}</p></div>;

export const Events: Component = () => {
  const [data, { mutate, refetch }] = createResource(fetchData);
  
  return (<For each={data()} fallback={<p>loading</p>}>{(detection) => <Event detection={detection} />}</For>);
}
